//===------------ PrinterLLVM.cpp - LLVM C++ code printer -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implementation of the LLVM C++ printer.
//
//===----------------------------------------------------------------------===//

#include "Printer.h"
#include "llvm/TableGen/TableGenBackend.h"

namespace llvm {

cl::OptionCategory PrinterLang("Select output language of backends");

static cl::opt<std::string>
    PrinterLangOpt("printerLang", cl::init("C++"),
                   cl::desc("Output language options: \"C++\" (default), "
                            "\"CCS\" (C Capstone style)"),
                   cl::cat(PrinterLang));

// Print a BitVector as a sequence of hex numbers using a little-endian mapping.
// Width is the number of bits per hex number.
void printBitVectorAsHex(raw_ostream &OS, const BitVector &Bits,
                         unsigned Width) {
  assert(Width <= 32 && "Width too large");
  unsigned const Digits = (Width + 3) / 4;
  for (unsigned I = 0, E = Bits.size(); I < E; I += Width) {
    unsigned Value = 0;
    for (unsigned J = 0; J != Width && I + J != E; ++J)
      Value |= Bits.test(I + J) << J;
    OS << format("0x%0*x, ", Digits, Value);
  }
}

void PrinterBitVectorEmitter::add(unsigned V) {
  if (V >= Values.size())
    Values.resize(((V / 8) + 1) * 8); // Round up to the next byte.
  Values[V] = true;
}

void PrinterBitVectorEmitter::print(raw_ostream &OS) {
  printBitVectorAsHex(OS, Values, 8);
}

//
// General PrinterLLVM methods
//

PrinterLLVM::PrinterLLVM(formatted_raw_ostream &OS) : OS(OS) {}
PrinterLLVM::~PrinterLLVM() {}

PrinterLanguage PrinterLLVM::getLanguage() {
  if (PrinterLangOpt == "C++")
    return PRINTER_LANG_CPP;
  if (PrinterLangOpt == "CCS")
    return PRINTER_LANG_CAPSTONE_C;

  PrintFatalNote("Unkown output language for printer selected.");
}

/// Prints `namespace <name> {` and `} // end namespace <name>` to the output
/// stream. If Name == "" it emits an anonymous namespace.
void PrinterLLVM::emitNamespace(std::string const &Name, bool Begin,
                                std::string const &Comment) const {
  if (Begin) {
    OS << "namespace " << Name;
    std::string const Bracket = (Name == "" ? "{" : " {");
    if (Comment != "")
      OS << Bracket << " // " << Comment << "\n";
    else
      OS << Bracket << "\n\n";
    return;
  }

  if (Name == "") {
    OS << "} // end anonymous namespace\n\n";
  } else {
    OS << "} // end namespace " << Name << "\n\n";
  }
}

/// Prints
/// ```
/// #ifdef <Name>
/// #undef <Name>
/// ```
/// and
/// `#endif // <Name>`
/// Used to control inclusion of a code block via a macro definition.
void PrinterLLVM::emitIncludeToggle(std::string const &Name, bool Begin) const {
  if (Begin) {
    OS << "\n#ifdef " << Name << "\n";
    OS << "#undef " << Name << "\n\n";
  } else {
    OS << "#endif // " << Name << "\n\n";
  }
}

void PrinterLLVM::regInfoEmitSourceFileHeader(std::string const &Desc) const {
  emitSourceFileHeader(Desc, OS);
}

// runEnums - Print out enum values for all of the registers.
void PrinterLLVM::regInfoEmitEnums(CodeGenTarget const &Target,
                                   CodeGenRegBank const &Bank) const {
  const auto &Registers = Bank.getRegisters();

  // Register enums are stored as uint16_t in the tables. Make sure we'll fit.
  assert(Registers.size() <= 0xffff && "Too many regs to fit in tables");

  StringRef const Namespace =
      Registers.front().TheDef->getValueAsString("Namespace");

  OS << "\n#ifdef GET_REGINFO_ENUM\n";
  OS << "#undef GET_REGINFO_ENUM\n\n";

  OS << "namespace llvm {\n\n";

  OS << "class MCRegisterClass;\n"
     << "extern const MCRegisterClass " << Target.getName()
     << "MCRegisterClasses[];\n\n";

  if (!Namespace.empty())
    OS << "namespace " << Namespace << " {\n";
  OS << "enum {\n  NoRegister,\n";

  for (const auto &Reg : Registers)
    OS << "  " << Reg.getName() << " = " << Reg.EnumValue << ",\n";
  assert(Registers.size() == Registers.back().EnumValue &&
         "Register enum value mismatch!");
  OS << "  NUM_TARGET_REGS // " << Registers.size() + 1 << "\n";
  OS << "};\n";
  if (!Namespace.empty())
    OS << "} // end namespace " << Namespace << "\n";

  const auto &RegisterClasses = Bank.getRegClasses();
  if (!RegisterClasses.empty()) {

    // RegisterClass enums are stored as uint16_t in the tables.
    assert(RegisterClasses.size() <= 0xffff &&
           "Too many register classes to fit in tables");

    OS << "\n// Register classes\n\n";
    if (!Namespace.empty())
      OS << "namespace " << Namespace << " {\n";
    OS << "enum {\n";
    for (const auto &RC : RegisterClasses)
      OS << "  " << RC.getName() << "RegClassID"
         << " = " << RC.EnumValue << ",\n";
    OS << "\n};\n";
    if (!Namespace.empty())
      OS << "} // end namespace " << Namespace << "\n\n";
  }

  const std::vector<Record *> &RegAltNameIndices =
      Target.getRegAltNameIndices();
  // If the only definition is the default NoRegAltName, we don't need to
  // emit anything.
  if (RegAltNameIndices.size() > 1) {
    OS << "\n// Register alternate name indices\n\n";
    if (!Namespace.empty())
      OS << "namespace " << Namespace << " {\n";
    OS << "enum {\n";
    for (unsigned I = 0, E = RegAltNameIndices.size(); I != E; ++I)
      OS << "  " << RegAltNameIndices[I]->getName() << ",\t// " << I << "\n";
    OS << "  NUM_TARGET_REG_ALT_NAMES = " << RegAltNameIndices.size() << "\n";
    OS << "};\n";
    if (!Namespace.empty())
      OS << "} // end namespace " << Namespace << "\n\n";
  }

  auto &SubRegIndices = Bank.getSubRegIndices();
  if (!SubRegIndices.empty()) {
    OS << "\n// Subregister indices\n\n";
    std::string const Namespace = SubRegIndices.front().getNamespace();
    if (!Namespace.empty())
      OS << "namespace " << Namespace << " {\n";
    OS << "enum : uint16_t {\n  NoSubRegister,\n";
    unsigned I = 0;
    for (const auto &Idx : SubRegIndices)
      OS << "  " << Idx.getName() << ",\t// " << ++I << "\n";
    OS << "  NUM_TARGET_SUBREGS\n};\n";
    if (!Namespace.empty())
      OS << "} // end namespace " << Namespace << "\n\n";
  }

  OS << "// Register pressure sets enum.\n";
  if (!Namespace.empty())
    OS << "namespace " << Namespace << " {\n";
  OS << "enum RegisterPressureSets {\n";
  unsigned const NumSets = Bank.getNumRegPressureSets();
  for (unsigned I = 0; I < NumSets; ++I) {
    const RegUnitSet &RegUnits = Bank.getRegSetAt(I);
    OS << "  " << RegUnits.Name << " = " << I << ",\n";
  }
  OS << "};\n";
  if (!Namespace.empty())
    OS << "} // end namespace " << Namespace << '\n';
  OS << '\n';

  OS << "} // end namespace llvm\n\n";
  OS << "#endif // GET_REGINFO_ENUM\n\n";
}

void PrinterLLVM::regInfoEmitRegDiffLists(
    std::string const TargetName,
    SequenceToOffsetTable<DiffVec> const &DiffSeqs) const {
  OS << "extern const MCPhysReg " << TargetName << "RegDiffLists[] = {\n";
  DiffSeqs.emit(OS, [](raw_ostream &OS, uint16_t Val) { OS << Val; });
  OS << "};\n\n";
}

static void regInfoPrintMask(raw_ostream &OS, LaneBitmask Val) {
  OS << "LaneBitmask(0x" << PrintLaneMask(Val) << ')';
}

void PrinterLLVM::regInfoEmitLaneMaskLists(
    std::string const TargetName,
    SequenceToOffsetTable<MaskVec> const &LaneMaskSeqs) const {
  OS << "extern const LaneBitmask " << TargetName << "LaneMaskLists[] = {\n";
  LaneMaskSeqs.emit(OS, regInfoPrintMask, "LaneBitmask::getAll()");
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitSubRegIdxLists(
    std::string const TargetName,
    SequenceToOffsetTable<SubRegIdxVec, deref<std::less<>>> const
        &SubRegIdxSeqs) const {
  OS << "extern const uint16_t " << TargetName << "SubRegIdxLists[] = {\n";
  SubRegIdxSeqs.emit(OS, [](raw_ostream &OS, const CodeGenSubRegIndex *Idx) {
    OS << Idx->EnumValue;
  });
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitSubRegIdxSizes(
    std::string const TargetName,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  OS << "extern const MCRegisterInfo::SubRegCoveredBits " << TargetName
     << "SubRegIdxRanges[] = {\n";
  OS << "  { " << (uint16_t)-1 << ", " << (uint16_t)-1 << " },\n";
  for (const auto &Idx : SubRegIndices) {
    OS << "  { " << Idx.Offset << ", " << Idx.Size << " },\t// "
       << Idx.getName() << "\n";
  }
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitSubRegStrTable(
    std::string const TargetName,
    SequenceToOffsetTable<std::string> const &RegStrings) const {
  RegStrings.emitStringLiteralDef(OS, Twine("extern const char ") + TargetName +
                                          "RegStrings[]");

  OS << "extern const MCRegisterDesc " << TargetName
     << "RegDesc[] = { // Descriptors\n";
  OS << "  { " << RegStrings.get("") << ", 0, 0, 0, 0, 0 },\n";
}

void PrinterLLVM::regInfoEmitRegDesc(
    SequenceToOffsetTable<MaskVec> const &LaneMaskSeqs,
    std::deque<CodeGenRegister> const &Regs,
    SequenceToOffsetTable<SubRegIdxVec, deref<std::less<>>> const
        &SubRegIdxSeqs,
    SequenceToOffsetTable<DiffVec> const &DiffSeqs,
    SmallVector<SubRegIdxVec, 4> const &SubRegIdxLists,
    SmallVector<DiffVec, 4> const &SubRegLists,
    SmallVector<DiffVec, 4> const &SuperRegLists,
    SmallVector<DiffVec, 4> const &RegUnitLists,
    SmallVector<unsigned, 4> const &RegUnitInitScale,
    SmallVector<MaskVec, 4> const &RegUnitLaneMasks,
    SequenceToOffsetTable<std::string> const &RegStrings) const {
  unsigned I = 0;
  for (const auto &Reg : Regs) {
    OS << "  { " << RegStrings.get(std::string(Reg.getName())) << ", "
       << DiffSeqs.get(SubRegLists[I]) << ", " << DiffSeqs.get(SuperRegLists[I])
       << ", " << SubRegIdxSeqs.get(SubRegIdxLists[I]) << ", "
       << (DiffSeqs.get(RegUnitLists[I]) * 16 + RegUnitInitScale[I]) << ", "
       << LaneMaskSeqs.get(RegUnitLaneMasks[I]) << " },\n";
    ++I;
  }
  OS << "};\n\n"; // End of register descriptors...
}

void PrinterLLVM::regInfoEmitRegUnitRoots(std::string const TargetName,
                                          CodeGenRegBank const &RegBank) const {
  OS << "extern const MCPhysReg " << TargetName << "RegUnitRoots[][2] = {\n";
  for (unsigned I = 0, E = RegBank.getNumNativeRegUnits(); I != E; ++I) {
    ArrayRef<const CodeGenRegister *> const Roots =
        RegBank.getRegUnit(I).getRoots();
    assert(!Roots.empty() && "All regunits must have a root register.");
    assert(Roots.size() <= 2 && "More than two roots not supported yet.");
    OS << "  { ";
    ListSeparator LS;
    for (const CodeGenRegister *R : Roots)
      OS << LS << getQualifiedName(R->TheDef);
    OS << " },\n";
  }
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitRegClasses(
    std::list<CodeGenRegisterClass> const &RegClasses,
    SequenceToOffsetTable<std::string> &RegClassStrings,
    CodeGenTarget const &Target) const {
  for (const auto &RC : RegClasses) {
    ArrayRef<Record *> const Order = RC.getOrder();

    // Give the register class a legal C name if it's anonymous.
    const std::string &Name = RC.getName();

    RegClassStrings.add(Name);

    // Emit the register list now (unless it would be a zero-length array).
    if (!Order.empty()) {
      OS << "  // " << Name << " Register Class...\n"
         << "  const MCPhysReg " << Name << "[] = {\n    ";
      for (Record *Reg : Order) {
        OS << getQualifiedName(Reg) << ", ";
      }
      OS << "\n  };\n\n";

      OS << "  // " << Name << " Bit set.\n"
         << "  const uint8_t " << Name << "Bits[] = {\n    ";
      PrinterBitVectorEmitter BVE;
      for (Record *Reg : Order) {
        BVE.add(Target.getRegBank().getReg(Reg)->EnumValue);
      }
      BVE.print(OS);
      OS << "\n  };\n\n";
    }
  }
}

void PrinterLLVM::regInfoEmitStrLiteralRegClasses(
    std::string const TargetName,
    SequenceToOffsetTable<std::string> const &RegClassStrings) const {
  RegClassStrings.emitStringLiteralDef(
      OS, Twine("extern const char ") + TargetName + "RegClassStrings[]");
}

void PrinterLLVM::regInfoEmitMCRegClassesTable(
    std::string const TargetName,
    std::list<CodeGenRegisterClass> const &RegClasses,
    SequenceToOffsetTable<std::string> &RegClassStrings) const {
  OS << "extern const MCRegisterClass " << TargetName
     << "MCRegisterClasses[] = {\n";

  for (const auto &RC : RegClasses) {
    ArrayRef<Record *> const Order = RC.getOrder();
    std::string const RCName = Order.empty() ? "nullptr" : RC.getName();
    std::string const RCBitsName =
        Order.empty() ? "nullptr" : RC.getName() + "Bits";
    std::string const RCBitsSize =
        Order.empty() ? "0" : "sizeof(" + RCBitsName + ")";
    assert(isInt<8>(RC.CopyCost) && "Copy cost too large.");
    uint32_t RegSize = 0;
    if (RC.RSI.isSimple())
      RegSize = RC.RSI.getSimple().RegSize;
    OS << "  { " << RCName << ", " << RCBitsName << ", "
       << RegClassStrings.get(RC.getName()) << ", " << RC.getOrder().size()
       << ", " << RCBitsSize << ", " << RC.getQualifiedName() + "RegClassID"
       << ", " << RegSize << ", " << RC.CopyCost << ", "
       << (RC.Allocatable ? "true" : "false") << " },\n";
  }

  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitRegEncodingTable(
    std::string const TargetName,
    std::deque<CodeGenRegister> const &Regs) const {
  OS << "extern const uint16_t " << TargetName;
  OS << "RegEncodingTable[] = {\n";
  // Add entry for NoRegister
  OS << "  0,\n";
  for (const auto &RE : Regs) {
    Record *Reg = RE.TheDef;
    BitsInit *BI = Reg->getValueAsBitsInit("HWEncoding");
    uint64_t Value = 0;
    for (unsigned I = 0, Ie = BI->getNumBits(); I != Ie; ++I) {
      if (BitInit *B = dyn_cast<BitInit>(BI->getBit(I)))
        Value |= (uint64_t)B->getValue() << I;
    }
    OS << "  " << Value << ",\n";
  }
  OS << "};\n"; // End of HW encoding table
}

void PrinterLLVM::regInfoEmitMCRegInfoInit(
    std::string const TargetName, CodeGenRegBank const &RegBank,
    std::deque<CodeGenRegister> const &Regs,
    std::list<CodeGenRegisterClass> const &RegClasses,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  OS << "static inline void Init" << TargetName
     << "MCRegisterInfo(MCRegisterInfo *RI, unsigned RA, "
     << "unsigned DwarfFlavour = 0, unsigned EHFlavour = 0, unsigned PC = 0) "
        "{\n"
     << "  RI->InitMCRegisterInfo(" << TargetName << "RegDesc, "
     << Regs.size() + 1 << ", RA, PC, " << TargetName << "MCRegisterClasses, "
     << RegClasses.size() << ", " << TargetName << "RegUnitRoots, "
     << RegBank.getNumNativeRegUnits() << ", " << TargetName << "RegDiffLists, "
     << TargetName << "LaneMaskLists, " << TargetName << "RegStrings, "
     << TargetName << "RegClassStrings, " << TargetName << "SubRegIdxLists, "
     << (std::distance(SubRegIndices.begin(), SubRegIndices.end()) + 1) << ",\n"
     << TargetName << "SubRegIdxRanges, " << TargetName
     << "RegEncodingTable);\n\n";
}

void PrinterLLVM::regInfoEmitInfoDwarfRegsRev(StringRef const &Namespace,
                                              DwarfRegNumsVecTy &DwarfRegNums,
                                              unsigned MaxLength,
                                              bool IsCtor) const {
  OS << "// " << Namespace << " Dwarf<->LLVM register mappings.\n";

  // Emit reverse information about the dwarf register numbers.
  for (unsigned j = 0; j < 2; ++j) {
    for (unsigned I = 0, E = MaxLength; I != E; ++I) {
      OS << "extern const MCRegisterInfo::DwarfLLVMRegPair " << Namespace;
      OS << (j == 0 ? "DwarfFlavour" : "EHFlavour");
      OS << I << "Dwarf2L[]";

      if (!IsCtor) {
        OS << " = {\n";

        // Store the mapping sorted by the LLVM reg num so lookup can be done
        // with a binary search.
        std::map<uint64_t, Record *> Dwarf2LMap;
        for (auto &DwarfRegNum : DwarfRegNums) {
          int DwarfRegNo = DwarfRegNum.second[I];
          if (DwarfRegNo < 0)
            continue;
          Dwarf2LMap[DwarfRegNo] = DwarfRegNum.first;
        }

        for (auto &I : Dwarf2LMap)
          OS << "  { " << I.first << "U, " << getQualifiedName(I.second)
             << " },\n";

        OS << "};\n";
      } else {
        OS << ";\n";
      }

      // We have to store the size in a const global, it's used in multiple
      // places.
      OS << "extern const unsigned " << Namespace
         << (j == 0 ? "DwarfFlavour" : "EHFlavour") << I << "Dwarf2LSize";
      if (!IsCtor)
        OS << " = std::size(" << Namespace
           << (j == 0 ? "DwarfFlavour" : "EHFlavour") << I << "Dwarf2L);\n\n";
      else
        OS << ";\n\n";
    }
  }
}

void PrinterLLVM::regInfoEmitInfoDwarfRegs(StringRef const &Namespace,
                                           DwarfRegNumsVecTy &DwarfRegNums,
                                           unsigned MaxLength,
                                           bool IsCtor) const {
  for (unsigned J = 0; J < 2; ++J) {
    for (unsigned I = 0, E = MaxLength; I != E; ++I) {
      OS << "extern const MCRegisterInfo::DwarfLLVMRegPair " << Namespace;
      OS << (J == 0 ? "DwarfFlavour" : "EHFlavour");
      OS << I << "L2Dwarf[]";
      if (!IsCtor) {
        OS << " = {\n";
        // Store the mapping sorted by the Dwarf reg num so lookup can be done
        // with a binary search.
        for (auto &DwarfRegNum : DwarfRegNums) {
          int RegNo = DwarfRegNum.second[I];
          if (RegNo == -1) // -1 is the default value, don't emit a mapping.
            continue;

          OS << "  { " << getQualifiedName(DwarfRegNum.first) << ", " << RegNo
             << "U },\n";
        }
        OS << "};\n";
      } else {
        OS << ";\n";
      }

      // We have to store the size in a const global, it's used in multiple
      // places.
      OS << "extern const unsigned " << Namespace
         << (J == 0 ? "DwarfFlavour" : "EHFlavour") << I << "L2DwarfSize";
      if (!IsCtor)
        OS << " = std::size(" << Namespace
           << (J == 0 ? "DwarfFlavour" : "EHFlavour") << I << "L2Dwarf);\n\n";
      else
        OS << ";\n\n";
    }
  }
}

void PrinterLLVM::regInfoEmitInfoRegMapping(StringRef const &Namespace,
                                            unsigned MaxLength,
                                            bool IsCtor) const {
  if (MaxLength == 0) {
    OS << "}\n\n";
    return;
  }

  // Emit reverse information about the dwarf register numbers.
  for (unsigned J = 0; J < 2; ++J) {
    OS << "  switch (";
    if (J == 0)
      OS << "DwarfFlavour";
    else
      OS << "EHFlavour";
    OS << ") {\n"
       << "  default:\n"
       << "    llvm_unreachable(\"Unknown DWARF flavour\");\n";

    for (unsigned I = 0, E = MaxLength; I != E; ++I) {
      OS << "  case " << I << ":\n";
      OS << "    ";
      if (!IsCtor)
        OS << "RI->";
      std::string Tmp;
      raw_string_ostream(Tmp)
          << Namespace << (J == 0 ? "DwarfFlavour" : "EHFlavour") << I
          << "Dwarf2L";
      OS << "mapDwarfRegsToLLVMRegs(" << Tmp << ", " << Tmp << "Size, ";
      if (J == 0)
        OS << "false";
      else
        OS << "true";
      OS << ");\n";
      OS << "    break;\n";
    }
    OS << "  }\n";
  }

  // Emit information about the dwarf register numbers.
  for (unsigned J = 0; J < 2; ++J) {
    OS << "  switch (";
    if (J == 0)
      OS << "DwarfFlavour";
    else
      OS << "EHFlavour";
    OS << ") {\n"
       << "  default:\n"
       << "    llvm_unreachable(\"Unknown DWARF flavour\");\n";

    for (unsigned I = 0, E = MaxLength; I != E; ++I) {
      OS << "  case " << I << ":\n";
      OS << "    ";
      if (!IsCtor)
        OS << "RI->";
      std::string Tmp;
      raw_string_ostream(Tmp)
          << Namespace << (J == 0 ? "DwarfFlavour" : "EHFlavour") << I
          << "L2Dwarf";
      OS << "mapLLVMRegsToDwarfRegs(" << Tmp << ", " << Tmp << "Size, ";
      if (J == 0)
        OS << "false";
      else
        OS << "true";
      OS << ");\n";
      OS << "    break;\n";
    }
    OS << "  }\n";
  }

  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitHeaderIncludes() const {
  OS << "#include \"llvm/CodeGen/TargetRegisterInfo.h\"\n\n";
}

void PrinterLLVM::regInfoEmitHeaderExternRegClasses(
    std::list<CodeGenRegisterClass> const &RegClasses) const {
  for (const auto &RC : RegClasses) {
    const std::string &Name = RC.getName();

    // Output the extern for the instance.
    OS << "  extern const TargetRegisterClass " << Name << "RegClass;\n";
  }
}

void PrinterLLVM::regInfoEmitHeaderDecl(std::string const &TargetName,
                                        std::string const &ClassName,
                                        bool SubRegsPresent,
                                        bool DeclareGetPhysRegBaseClass) const {
  OS << "class " << TargetName << "FrameLowering;\n\n";

  OS << "struct " << ClassName << " : public TargetRegisterInfo {\n"
     << "  explicit " << ClassName
     << "(unsigned RA, unsigned D = 0, unsigned E = 0,\n"
     << "      unsigned PC = 0, unsigned HwMode = 0);\n";
  if (SubRegsPresent) {
    OS << "  unsigned composeSubRegIndicesImpl"
       << "(unsigned, unsigned) const override;\n"
       << "  LaneBitmask composeSubRegIndexLaneMaskImpl"
       << "(unsigned, LaneBitmask) const override;\n"
       << "  LaneBitmask reverseComposeSubRegIndexLaneMaskImpl"
       << "(unsigned, LaneBitmask) const override;\n"
       << "  const TargetRegisterClass *getSubClassWithSubReg"
       << "(const TargetRegisterClass *, unsigned) const override;\n"
       << "  const TargetRegisterClass *getSubRegisterClass"
       << "(const TargetRegisterClass *, unsigned) const override;\n";
  }
  OS << "  const RegClassWeight &getRegClassWeight("
     << "const TargetRegisterClass *RC) const override;\n"
     << "  unsigned getRegUnitWeight(unsigned RegUnit) const override;\n"
     << "  unsigned getNumRegPressureSets() const override;\n"
     << "  const char *getRegPressureSetName(unsigned Idx) const override;\n"
     << "  unsigned getRegPressureSetLimit(const MachineFunction &MF, unsigned "
        "Idx) const override;\n"
     << "  const int *getRegClassPressureSets("
     << "const TargetRegisterClass *RC) const override;\n"
     << "  const int *getRegUnitPressureSets("
     << "unsigned RegUnit) const override;\n"
     << "  ArrayRef<const char *> getRegMaskNames() const override;\n"
     << "  ArrayRef<const uint32_t *> getRegMasks() const override;\n"
     << "  bool isGeneralPurposeRegister(const MachineFunction &, "
     << "MCRegister) const override;\n"
     << "  bool isFixedRegister(const MachineFunction &, "
     << "MCRegister) const override;\n"
     << "  bool isArgumentRegister(const MachineFunction &, "
     << "MCRegister) const override;\n"
     << "  bool isConstantPhysReg(MCRegister PhysReg) const override final;\n"
     << "  /// Devirtualized TargetFrameLowering.\n"
     << "  static const " << TargetName << "FrameLowering *getFrameLowering(\n"
     << "      const MachineFunction &MF);\n";
  if (DeclareGetPhysRegBaseClass) {
    OS << "  const TargetRegisterClass *getPhysRegBaseClass(MCRegister Reg) "
          "const override;\n";
  }
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitExternRegClassesArr(
    std::string const &TargetName) const {
  OS << "extern const MCRegisterClass " << TargetName
     << "MCRegisterClasses[];\n";
}

static void printSimpleValueType(raw_ostream &OS, MVT::SimpleValueType VT) {
  OS << getEnumName(VT);
}

void PrinterLLVM::regInfoEmitVTSeqs(
    SequenceToOffsetTable<std::vector<MVT::SimpleValueType>> const &VTSeqs)
    const {
  OS << "\nstatic const MVT::SimpleValueType VTLists[] = {\n";
  VTSeqs.emit(OS, printSimpleValueType, "MVT::Other");
  OS << "};\n";
}

static void printMask(raw_ostream &OS, LaneBitmask Val) {
  OS << "LaneBitmask(0x" << PrintLaneMask(Val) << ')';
}

void PrinterLLVM::regInfoEmitSubRegIdxTable(
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  OS << "\nstatic const char *SubRegIndexNameTable[] = { \"";

  for (const auto &Idx : SubRegIndices) {
    OS << Idx.getName();
    OS << "\", \"";
  }
  OS << "\" };\n\n";

  // Emit SubRegIndex lane masks, including 0.
  OS << "\nstatic const LaneBitmask SubRegIndexLaneMaskTable[] = {\n  "
        "LaneBitmask::getAll(),\n";
  for (const auto &Idx : SubRegIndices) {
    printMask(OS << "  ", Idx.LaneMask);
    OS << ", // " << Idx.getName() << '\n';
  }
  OS << " };\n\n";

  OS << "\n";
}

void PrinterLLVM::regInfoEmitRegClassInfoTable(
    std::list<CodeGenRegisterClass> const &RegClasses,
    SequenceToOffsetTable<std::vector<MVT::SimpleValueType>> const &VTSeqs,
    CodeGenHwModes const &CGH, unsigned NumModes) const {
  OS << "\nstatic const TargetRegisterInfo::RegClassInfo RegClassInfos[]"
     << " = {\n";
  for (unsigned M = 0; M < NumModes; ++M) {
    unsigned EV = 0;
    OS << "  // Mode = " << M << " (";
    if (M == 0)
      OS << "Default";
    else
      OS << CGH.getMode(M).Name;
    OS << ")\n";
    for (const auto &RC : RegClasses) {
      assert(RC.EnumValue == EV && "Unexpected order of register classes");
      ++EV;
      (void)EV;
      const RegSizeInfo &RI = RC.RSI.get(M);
      OS << "  { " << RI.RegSize << ", " << RI.SpillSize << ", "
         << RI.SpillAlignment;
      std::vector<MVT::SimpleValueType> VTs;
      for (const ValueTypeByHwMode &VVT : RC.VTs)
        VTs.push_back(VVT.get(M).SimpleTy);
      OS << ", VTLists+" << VTSeqs.get(VTs) << " },    // " << RC.getName()
         << '\n';
    }
  }
  OS << "};\n";

  OS << "\nstatic const TargetRegisterClass *const "
     << "NullRegClasses[] = { nullptr };\n\n";
}

void PrinterLLVM::regInfoEmitSubClassMaskTable(
    std::list<CodeGenRegisterClass> const &RegClasses,
    SmallVector<IdxList, 8> &SuperRegIdxLists,
    SequenceToOffsetTable<IdxList, deref<std::less<>>> &SuperRegIdxSeqs,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices,
    BitVector &MaskBV) const {
  for (const auto &RC : RegClasses) {
    OS << "static const uint32_t " << RC.getName() << "SubClassMask[] = {\n  ";
    printBitVectorAsHex(OS, RC.getSubClasses(), 32);

    // Emit super-reg class masks for any relevant SubRegIndices that can
    // project into RC.
    IdxList &SRIList = SuperRegIdxLists[RC.EnumValue];
    for (auto &Idx : SubRegIndices) {
      MaskBV.reset();
      RC.getSuperRegClasses(&Idx, MaskBV);
      if (MaskBV.none())
        continue;
      SRIList.push_back(&Idx);
      OS << "\n  ";
      printBitVectorAsHex(OS, MaskBV, 32);
      OS << "// " << Idx.getName();
    }
    SuperRegIdxSeqs.add(SRIList);
    OS << "\n};\n\n";
  }
}

static void printSubRegIndex(raw_ostream &OS, const CodeGenSubRegIndex *Idx) {
  OS << Idx->EnumValue;
}

void PrinterLLVM::regInfoEmitSuperRegIdxSeqsTable(
    SequenceToOffsetTable<IdxList, deref<std::less<>>> const &SuperRegIdxSeqs)
    const {
  OS << "static const uint16_t SuperRegIdxSeqs[] = {\n";
  SuperRegIdxSeqs.emit(OS, printSubRegIndex);
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitSuperClassesTable(
    std::list<CodeGenRegisterClass> const &RegClasses) const {
  for (const auto &RC : RegClasses) {
    ArrayRef<CodeGenRegisterClass *> const Supers = RC.getSuperClasses();

    // Skip classes without supers.  We can reuse NullRegClasses.
    if (Supers.empty())
      continue;

    OS << "static const TargetRegisterClass *const " << RC.getName()
       << "Superclasses[] = {\n";
    for (const auto *Super : Supers)
      OS << "  &" << Super->getQualifiedName() << "RegClass,\n";
    OS << "  nullptr\n};\n\n";
  }
}

void PrinterLLVM::regInfoEmitRegClassMethods(
    std::list<CodeGenRegisterClass> const &RegClasses,
    std::string const &TargetName) const {
  for (const auto &RC : RegClasses) {
    if (!RC.AltOrderSelect.empty()) {
      OS << "\nstatic inline unsigned " << RC.getName()
         << "AltOrderSelect(const MachineFunction &MF) {" << RC.AltOrderSelect
         << "}\n\n"
         << "static ArrayRef<MCPhysReg> " << RC.getName()
         << "GetRawAllocationOrder(const MachineFunction &MF) {\n";
      for (unsigned Oi = 1, Oe = RC.getNumOrders(); Oi != Oe; ++Oi) {
        ArrayRef<Record *> const Elems = RC.getOrder(Oi);
        if (!Elems.empty()) {
          OS << "  static const MCPhysReg AltOrder" << Oi << "[] = {";
          for (unsigned Elem = 0; Elem != Elems.size(); ++Elem)
            OS << (Elem ? ", " : " ") << getQualifiedName(Elems[Elem]);
          OS << " };\n";
        }
      }
      OS << "  const MCRegisterClass &MCR = " << TargetName
         << "MCRegisterClasses[" << RC.getQualifiedName() + "RegClassID];\n"
         << "  const ArrayRef<MCPhysReg> Order[] = {\n"
         << "    makeArrayRef(MCR.begin(), MCR.getNumRegs()";
      for (unsigned Oi = 1, Oe = RC.getNumOrders(); Oi != Oe; ++Oi)
        if (RC.getOrder(Oi).empty())
          OS << "),\n    ArrayRef<MCPhysReg>(";
        else
          OS << "),\n    makeArrayRef(AltOrder" << Oi;
      OS << ")\n  };\n  const unsigned Select = " << RC.getName()
         << "AltOrderSelect(MF);\n  assert(Select < " << RC.getNumOrders()
         << ");\n  return Order[Select];\n}\n";
    }
  }
}

void PrinterLLVM::regInfomitRegClassInstances(
    std::list<CodeGenRegisterClass> const &RegClasses,
    SequenceToOffsetTable<IdxList, deref<std::less<>>> const &SuperRegIdxSeqs,
    SmallVector<IdxList, 8> const &SuperRegIdxLists,
    std::string const &TargetName) const {
  for (const auto &RC : RegClasses) {
    OS << "  extern const TargetRegisterClass " << RC.getName()
       << "RegClass = {\n    " << '&' << TargetName << "MCRegisterClasses["
       << RC.getName() << "RegClassID],\n    " << RC.getName()
       << "SubClassMask,\n    SuperRegIdxSeqs + "
       << SuperRegIdxSeqs.get(SuperRegIdxLists[RC.EnumValue]) << ",\n    ";
    printMask(OS, RC.LaneMask);
    OS << ",\n    " << (unsigned)RC.AllocationPriority << ",\n    "
       << (RC.GlobalPriority ? "true" : "false") << ",\n    "
       << format("0x%02x", RC.TSFlags) << ", /* TSFlags */\n    "
       << (RC.HasDisjunctSubRegs ? "true" : "false")
       << ", /* HasDisjunctSubRegs */\n    "
       << (RC.CoveredBySubRegs ? "true" : "false")
       << ", /* CoveredBySubRegs */\n    ";
    if (RC.getSuperClasses().empty())
      OS << "NullRegClasses,\n    ";
    else
      OS << RC.getName() << "Superclasses,\n    ";
    if (RC.AltOrderSelect.empty())
      OS << "nullptr\n";
    else
      OS << RC.getName() << "GetRawAllocationOrder\n";
    OS << "  };\n\n";
  }
}

void PrinterLLVM::regInfoEmitRegClassTable(
    std::list<CodeGenRegisterClass> const &RegClasses) const {
  OS << "  const TargetRegisterClass *const RegisterClasses[] = {\n";
  for (const auto &RC : RegClasses)
    OS << "    &" << RC.getQualifiedName() << "RegClass,\n";
  OS << "  };\n";
}

void PrinterLLVM::regInfoEmitCostPerUseTable(
    std::vector<unsigned> const &AllRegCostPerUse, unsigned NumRegCosts) const {
  OS << "\nstatic const uint8_t "
     << "CostPerUseTable[] = { \n";
  for (unsigned int I = 0; I < NumRegCosts; ++I) {
    for (unsigned J = I, E = AllRegCostPerUse.size(); J < E; J += NumRegCosts)
      OS << AllRegCostPerUse[J] << ", ";
  }
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitInAllocatableClassTable(
    llvm::BitVector const &InAllocClass) const {
  OS << "\nstatic const bool "
     << "InAllocatableClassTable[] = { \n";
  for (unsigned I = 0, E = InAllocClass.size(); I < E; ++I) {
    OS << (InAllocClass[I] ? "true" : "false") << ", ";
  }
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitRegExtraDesc(std::string const &TargetName,
                                          unsigned NumRegCosts) const {
  OS << "\nstatic const TargetRegisterInfoDesc " << TargetName
     << "RegInfoDesc = { // Extra Descriptors\n";
  OS << "CostPerUseTable, " << NumRegCosts << ", "
     << "InAllocatableClassTable";
  OS << "};\n\n";
}

void PrinterLLVM::regInfoEmitSubClassSubRegGetter(
    std::string const &ClassName, unsigned SubRegIndicesSize,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices,
    std::list<CodeGenRegisterClass> const &RegClasses,
    CodeGenRegBank &RegBank) const {
  // Emit getSubClassWithSubReg.
  OS << "const TargetRegisterClass *" << ClassName
     << "::getSubClassWithSubReg(const TargetRegisterClass *RC, unsigned Idx)"
     << " const {\n";
  // Use the smallest type that can hold a regclass ID with room for a
  // sentinel.
  if (RegClasses.size() <= UINT8_MAX)
    OS << "  static const uint8_t Table[";
  else if (RegClasses.size() <= UINT16_MAX)
    OS << "  static const uint16_t Table[";
  else
    PrintFatalError("Too many register classes.");
  OS << RegClasses.size() << "][" << SubRegIndicesSize << "] = {\n";
  for (const auto &RC : RegClasses) {
    OS << "    {\t// " << RC.getName() << "\n";
    for (auto &Idx : SubRegIndices) {
      if (CodeGenRegisterClass *SRC = RC.getSubClassWithSubReg(&Idx))
        OS << "      " << SRC->EnumValue + 1 << ",\t// " << Idx.getName()
           << " -> " << SRC->getName() << "\n";
      else
        OS << "      0,\t// " << Idx.getName() << "\n";
    }
    OS << "    },\n";
  }
  OS << "  };\n  assert(RC && \"Missing regclass\");\n"
     << "  if (!Idx) return RC;\n  --Idx;\n"
     << "  assert(Idx < " << SubRegIndicesSize << " && \"Bad subreg\");\n"
     << "  unsigned TV = Table[RC->getID()][Idx];\n"
     << "  return TV ? getRegClass(TV - 1) : nullptr;\n}\n\n";

  // Emit getSubRegisterClass
  OS << "const TargetRegisterClass *" << ClassName
     << "::getSubRegisterClass(const TargetRegisterClass *RC, unsigned Idx)"
     << " const {\n";

  // Use the smallest type that can hold a regclass ID with room for a
  // sentinel.
  if (RegClasses.size() <= UINT8_MAX)
    OS << "  static const uint8_t Table[";
  else if (RegClasses.size() <= UINT16_MAX)
    OS << "  static const uint16_t Table[";
  else
    PrintFatalError("Too many register classes.");

  OS << RegClasses.size() << "][" << SubRegIndicesSize << "] = {\n";

  for (const auto &RC : RegClasses) {
    OS << "    {\t// " << RC.getName() << '\n';
    for (auto &Idx : SubRegIndices) {
      std::optional<std::pair<CodeGenRegisterClass *, CodeGenRegisterClass *>>
          MatchingSubClass = RC.getMatchingSubClassWithSubRegs(RegBank, &Idx);

      unsigned EnumValue = 0;
      if (MatchingSubClass) {
        CodeGenRegisterClass *SubRegClass = MatchingSubClass->second;
        EnumValue = SubRegClass->EnumValue + 1;
      }

      OS << "      " << EnumValue << ",\t// " << RC.getName() << ':'
         << Idx.getName();

      if (MatchingSubClass) {
        CodeGenRegisterClass *SubRegClass = MatchingSubClass->second;
        OS << " -> " << SubRegClass->getName();
      }

      OS << '\n';
    }

    OS << "    },\n";
  }
  OS << "  };\n  assert(RC && \"Missing regclass\");\n"
     << "  if (!Idx) return RC;\n  --Idx;\n"
     << "  assert(Idx < " << SubRegIndicesSize << " && \"Bad subreg\");\n"
     << "  unsigned TV = Table[RC->getID()][Idx];\n"
     << "  return TV ? getRegClass(TV - 1) : nullptr;\n}\n\n";
}

void PrinterLLVM::regInfoEmitRegClassWeight(
    CodeGenRegBank const &RegBank, std::string const &ClassName) const {
  OS << "/// Get the weight in units of pressure for this register class.\n"
     << "const RegClassWeight &" << ClassName << "::\n"
     << "getRegClassWeight(const TargetRegisterClass *RC) const {\n"
     << "  static const RegClassWeight RCWeightTable[] = {\n";
  for (const auto &RC : RegBank.getRegClasses()) {
    const CodeGenRegister::Vec &Regs = RC.getMembers();
    OS << "    {" << RC.getWeight(RegBank) << ", ";
    if (Regs.empty() || RC.Artificial)
      OS << '0';
    else {
      std::vector<unsigned> RegUnits;
      RC.buildRegUnitSet(RegBank, RegUnits);
      OS << RegBank.getRegUnitSetWeight(RegUnits);
    }
    OS << "},  \t// " << RC.getName() << "\n";
  }
  OS << "  };\n"
     << "  return RCWeightTable[RC->getID()];\n"
     << "}\n\n";
}

void PrinterLLVM::regInfoEmitRegUnitWeight(CodeGenRegBank const &RegBank,
                                           std::string const &ClassName,
                                           bool RegUnitsHaveUnitWeight) const {
  OS << "/// Get the weight in units of pressure for this register unit.\n"
     << "unsigned " << ClassName << "::\n"
     << "getRegUnitWeight(unsigned RegUnit) const {\n"
     << "  assert(RegUnit < " << RegBank.getNumNativeRegUnits()
     << " && \"invalid register unit\");\n";
  if (!RegUnitsHaveUnitWeight) {
    OS << "  static const uint8_t RUWeightTable[] = {\n    ";
    for (unsigned UnitIdx = 0, UnitEnd = RegBank.getNumNativeRegUnits();
         UnitIdx < UnitEnd; ++UnitIdx) {
      const RegUnit &RU = RegBank.getRegUnit(UnitIdx);
      assert(RU.Weight < 256 && "RegUnit too heavy");
      OS << RU.Weight << ", ";
    }
    OS << "};\n"
       << "  return RUWeightTable[RegUnit];\n";
  } else {
    OS << "  // All register units have unit weight.\n"
       << "  return 1;\n";
  }
  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitGetNumRegPressureSets(std::string const &ClassName,
                                                   unsigned NumSets) const {
  OS << "\n"
     << "// Get the number of dimensions of register pressure.\n"
     << "unsigned " << ClassName << "::getNumRegPressureSets() const {\n"
     << "  return " << NumSets << ";\n}\n\n";
}

void PrinterLLVM::regInfoEmitGetRegPressureTables(CodeGenRegBank const &RegBank,
                                                  std::string const &ClassName,
                                                  unsigned NumSets) const {
  OS << "// Get the name of this register unit pressure set.\n"
     << "const char *" << ClassName << "::\n"
     << "getRegPressureSetName(unsigned Idx) const {\n"
     << "  static const char *PressureNameTable[] = {\n";
  unsigned MaxRegUnitWeight = 0;
  for (unsigned I = 0; I < NumSets; ++I) {
    const RegUnitSet &RegUnits = RegBank.getRegSetAt(I);
    MaxRegUnitWeight = std::max(MaxRegUnitWeight, RegUnits.Weight);
    OS << "    \"" << RegUnits.Name << "\",\n";
  }
  OS << "  };\n"
     << "  return PressureNameTable[Idx];\n"
     << "}\n\n";

  OS << "// Get the register unit pressure limit for this dimension.\n"
     << "// This limit must be adjusted dynamically for reserved registers.\n"
     << "unsigned " << ClassName << "::\n"
     << "getRegPressureSetLimit(const MachineFunction &MF, unsigned Idx) const "
        "{\n"
     << "  static const " << getMinimalTypeForRange(MaxRegUnitWeight, 32)
     << " PressureLimitTable[] = {\n";
  for (unsigned I = 0; I < NumSets; ++I) {
    const RegUnitSet &RegUnits = RegBank.getRegSetAt(I);
    OS << "    " << RegUnits.Weight << ",  \t// " << I << ": " << RegUnits.Name
       << "\n";
  }
  OS << "  };\n"
     << "  return PressureLimitTable[Idx];\n"
     << "}\n\n";
}

static void printInt(raw_ostream &OS, int Val) { OS << Val; }

void PrinterLLVM::regInfoEmitRCSetsTable(
    std::string const &ClassName, unsigned NumRCs,
    SequenceToOffsetTable<std::vector<int>> const &PSetsSeqs,
    std::vector<std::vector<int>> const &PSets) const {
  OS << "/// Table of pressure sets per register class or unit.\n"
     << "static const int RCSetsTable[] = {\n";
  PSetsSeqs.emit(OS, printInt, "-1");
  OS << "};\n\n";

  OS << "/// Get the dimensions of register pressure impacted by this "
     << "register class.\n"
     << "/// Returns a -1 terminated array of pressure set IDs\n"
     << "const int *" << ClassName << "::\n"
     << "getRegClassPressureSets(const TargetRegisterClass *RC) const {\n";
  OS << "  static const " << getMinimalTypeForRange(PSetsSeqs.size() - 1, 32)
     << " RCSetStartTable[] = {\n    ";
  for (unsigned I = 0, E = NumRCs; I != E; ++I) {
    OS << PSetsSeqs.get(PSets[I]) << ",";
  }
  OS << "};\n"
     << "  return &RCSetsTable[RCSetStartTable[RC->getID()]];\n"
     << "}\n\n";
}

void PrinterLLVM::regInfoEmitGetRegUnitPressureSets(
    SequenceToOffsetTable<std::vector<int>> const &PSetsSeqs,
    CodeGenRegBank const &RegBank, std::string const &ClassName,
    std::vector<std::vector<int>> const &PSets) const {
  OS << "/// Get the dimensions of register pressure impacted by this "
     << "register unit.\n"
     << "/// Returns a -1 terminated array of pressure set IDs\n"
     << "const int *" << ClassName << "::\n"
     << "getRegUnitPressureSets(unsigned RegUnit) const {\n"
     << "  assert(RegUnit < " << RegBank.getNumNativeRegUnits()
     << " && \"invalid register unit\");\n";
  OS << "  static const " << getMinimalTypeForRange(PSetsSeqs.size() - 1, 32)
     << " RUSetStartTable[] = {\n    ";
  for (unsigned UnitIdx = 0, UnitEnd = RegBank.getNumNativeRegUnits();
       UnitIdx < UnitEnd; ++UnitIdx) {
    OS << PSetsSeqs.get(PSets[RegBank.getRegUnit(UnitIdx).RegClassUnitSetsIdx])
       << ",";
  }
  OS << "};\n"
     << "  return &RCSetsTable[RUSetStartTable[RegUnit]];\n"
     << "}\n\n";
}

void PrinterLLVM::regInfoEmitExternTableDecl(
    std::string const &TargetName) const {
  OS << "extern const MCRegisterDesc " << TargetName << "RegDesc[];\n";
  OS << "extern const MCPhysReg " << TargetName << "RegDiffLists[];\n";
  OS << "extern const LaneBitmask " << TargetName << "LaneMaskLists[];\n";
  OS << "extern const char " << TargetName << "RegStrings[];\n";
  OS << "extern const char " << TargetName << "RegClassStrings[];\n";
  OS << "extern const MCPhysReg " << TargetName << "RegUnitRoots[][2];\n";
  OS << "extern const uint16_t " << TargetName << "SubRegIdxLists[];\n";
  OS << "extern const MCRegisterInfo::SubRegCoveredBits " << TargetName
     << "SubRegIdxRanges[];\n";
  OS << "extern const uint16_t " << TargetName << "RegEncodingTable[];\n";
}

void PrinterLLVM::regInfoEmitRegClassInit(
    std::string const &TargetName, std::string const &ClassName,
    CodeGenRegBank const &RegBank,
    std::list<CodeGenRegisterClass> const &RegClasses,
    std::deque<CodeGenRegister> const &Regs, unsigned SubRegIndicesSize) const {
  OS << ClassName << "::\n"
     << ClassName
     << "(unsigned RA, unsigned DwarfFlavour, unsigned EHFlavour,\n"
        "      unsigned PC, unsigned HwMode)\n"
     << "  : TargetRegisterInfo(&" << TargetName << "RegInfoDesc"
     << ", RegisterClasses, RegisterClasses+" << RegClasses.size() << ",\n"
     << "             SubRegIndexNameTable, SubRegIndexLaneMaskTable,\n"
     << "             ";
  printMask(OS, RegBank.CoveringLanes);
  OS << ", RegClassInfos, HwMode) {\n"
     << "  InitMCRegisterInfo(" << TargetName << "RegDesc, " << Regs.size() + 1
     << ", RA, PC,\n                     " << TargetName
     << "MCRegisterClasses, " << RegClasses.size() << ",\n"
     << "                     " << TargetName << "RegUnitRoots,\n"
     << "                     " << RegBank.getNumNativeRegUnits() << ",\n"
     << "                     " << TargetName << "RegDiffLists,\n"
     << "                     " << TargetName << "LaneMaskLists,\n"
     << "                     " << TargetName << "RegStrings,\n"
     << "                     " << TargetName << "RegClassStrings,\n"
     << "                     " << TargetName << "SubRegIdxLists,\n"
     << "                     " << SubRegIndicesSize + 1 << ",\n"
     << "                     " << TargetName << "SubRegIdxRanges,\n"
     << "                     " << TargetName << "RegEncodingTable);\n\n";
}

void PrinterLLVM::regInfoEmitSaveListTable(
    Record const *CSRSet, SetTheory::RecVec const *Regs) const {
  OS << "static const MCPhysReg " << CSRSet->getName() << "_SaveList[] = { ";
  for (unsigned R = 0, Re = Regs->size(); R != Re; ++R)
    OS << getQualifiedName((*Regs)[R]) << ", ";
  OS << "0 };\n";
}

void PrinterLLVM::regInfoEmitRegMaskTable(std::string const &CSRSetName,
                                          BitVector &Covered) const {
  OS << "static const uint32_t " << CSRSetName << "_RegMask[] = { ";
  printBitVectorAsHex(OS, Covered, 32);
  OS << "};\n";
}

void PrinterLLVM::regInfoEmitGetRegMasks(std::vector<Record *> const &CSRSets,
                                         std::string const &ClassName) const {
  OS << "ArrayRef<const uint32_t *> " << ClassName
     << "::getRegMasks() const {\n";
  if (!CSRSets.empty()) {
    OS << "  static const uint32_t *const Masks[] = {\n";
    for (Record *CSRSet : CSRSets)
      OS << "    " << CSRSet->getName() << "_RegMask,\n";
    OS << "  };\n";
    OS << "  return makeArrayRef(Masks);\n";
  } else {
    OS << "  return std::nullopt;\n";
  }
  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitGPRCheck(
    std::string const &ClassName,
    std::list<CodeGenRegisterCategory> const &RegCategories) const {
  OS << "bool " << ClassName << "::\n"
     << "isGeneralPurposeRegister(const MachineFunction &MF, "
     << "MCRegister PhysReg) const {\n"
     << "  return\n";
  for (const CodeGenRegisterCategory &Category : RegCategories)
    if (Category.getName() == "GeneralPurposeRegisters") {
      for (const CodeGenRegisterClass *RC : Category.getClasses())
        OS << "      " << RC->getQualifiedName()
           << "RegClass.contains(PhysReg) ||\n";
      break;
    }
  OS << "      false;\n";
  OS << "}\n\n";
}
void PrinterLLVM::regInfoEmitFixedRegCheck(
    std::string const &ClassName,
    std::list<CodeGenRegisterCategory> const &RegCategories) const {
  OS << "bool " << ClassName << "::\n"
     << "isFixedRegister(const MachineFunction &MF, "
     << "MCRegister PhysReg) const {\n"
     << "  return\n";
  for (const CodeGenRegisterCategory &Category : RegCategories)
    if (Category.getName() == "FixedRegisters") {
      for (const CodeGenRegisterClass *RC : Category.getClasses())
        OS << "      " << RC->getQualifiedName()
           << "RegClass.contains(PhysReg) ||\n";
      break;
    }
  OS << "      false;\n";
  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitArgRegCheck(
    std::string const &ClassName,
    std::list<CodeGenRegisterCategory> const &RegCategories) const {
  OS << "bool " << ClassName << "::\n"
     << "isArgumentRegister(const MachineFunction &MF, "
     << "MCRegister PhysReg) const {\n"
     << "  return\n";
  for (const CodeGenRegisterCategory &Category : RegCategories)
    if (Category.getName() == "ArgumentRegisters") {
      for (const CodeGenRegisterClass *RC : Category.getClasses())
        OS << "      " << RC->getQualifiedName()
           << "RegClass.contains(PhysReg) ||\n";
      break;
    }
  OS << "      false;\n";
  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitIsConstantPhysReg(
    std::deque<CodeGenRegister> const &Regs,
    std::string const &ClassName) const {
  OS << "bool " << ClassName << "::\n"
     << "isConstantPhysReg(MCRegister PhysReg) const {\n"
     << "  return\n";
  for (const auto &Reg : Regs)
    if (Reg.Constant)
      OS << "      PhysReg == " << getQualifiedName(Reg.TheDef) << " ||\n";
  OS << "      false;\n";
  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitGetRegMaskNames(
    std::vector<Record *> const &CSRSets, std::string const &ClassName) const {
  OS << "ArrayRef<const char *> " << ClassName
     << "::getRegMaskNames() const {\n";
  if (!CSRSets.empty()) {
    OS << "  static const char *Names[] = {\n";
    for (Record *CSRSet : CSRSets)
      OS << "    " << '"' << CSRSet->getName() << '"' << ",\n";
    OS << "  };\n";
    OS << "  return makeArrayRef(Names);\n";
  } else {
    OS << "  return std::nullopt;\n";
  }
  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitGetFrameLowering(
    std::string const &TargetName) const {
  OS << "const " << TargetName << "FrameLowering *\n"
     << TargetName
     << "GenRegisterInfo::getFrameLowering(const MachineFunction &MF) {\n"
     << "  return static_cast<const " << TargetName << "FrameLowering *>(\n"
     << "      MF.getSubtarget().getFrameLowering());\n"
     << "}\n\n";
}

void PrinterLLVM::regInfoEmitComposeSubRegIndicesImplHead(
    std::string const &ClName) const {
  OS << "unsigned " << ClName
     << "::composeSubRegIndicesImpl(unsigned IdxA, unsigned IdxB) const {\n";
}

void PrinterLLVM::regInfoEmitComposeSubRegIndicesImplBody(
    SmallVector<SmallVector<CodeGenSubRegIndex *, 4>, 4> const &Rows,
    unsigned SubRegIndicesSize, SmallVector<unsigned, 4> const &RowMap) const {
  if (Rows.size() > 1) {
    OS << "  static const " << getMinimalTypeForRange(Rows.size(), 32)
       << " RowMap[" << SubRegIndicesSize << "] = {\n    ";
    for (unsigned I = 0, E = SubRegIndicesSize; I != E; ++I)
      OS << RowMap[I] << ", ";
    OS << "\n  };\n";
  }

  // Output the rows.
  OS << "  static const " << getMinimalTypeForRange(SubRegIndicesSize + 1, 32)
     << " Rows[" << Rows.size() << "][" << SubRegIndicesSize << "] = {\n";
  for (unsigned R = 0, Re = Rows.size(); R != Re; ++R) {
    OS << "    { ";
    for (unsigned I = 0, E = SubRegIndicesSize; I != E; ++I)
      if (Rows[R][I])
        OS << Rows[R][I]->getQualifiedName() << ", ";
      else
        OS << "0, ";
    OS << "},\n";
  }
  OS << "  };\n\n";

  OS << "  --IdxA; assert(IdxA < " << SubRegIndicesSize << "); (void) IdxA;\n"
     << "  --IdxB; assert(IdxB < " << SubRegIndicesSize << ");\n";
  if (Rows.size() > 1)
    OS << "  return Rows[RowMap[IdxA]][IdxB];\n";
  else
    OS << "  return Rows[0][IdxB];\n";
  OS << "}\n\n";
}

void PrinterLLVM::regInfoEmitLaneMaskComposeSeq(
    SmallVector<SmallVector<MaskRolPair, 1>, 4> const &Sequences,
    SmallVector<unsigned, 4> const &SubReg2SequenceIndexMap,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  OS << "  struct MaskRolOp {\n"
        "    LaneBitmask Mask;\n"
        "    uint8_t  RotateLeft;\n"
        "  };\n"
        "  static const MaskRolOp LaneMaskComposeSequences[] = {\n";
  unsigned Idx = 0;
  for (size_t S = 0, Se = Sequences.size(); S != Se; ++S) {
    OS << "    ";
    const SmallVectorImpl<MaskRolPair> &Sequence = Sequences[S];
    for (size_t P = 0, Pe = Sequence.size(); P != Pe; ++P) {
      const MaskRolPair &MRP = Sequence[P];
      printMask(OS << "{ ", MRP.Mask);
      OS << format(", %2u }, ", MRP.RotateLeft);
    }
    OS << "{ LaneBitmask::getNone(), 0 }";
    if (S + 1 != Se)
      OS << ", ";
    OS << "  // Sequence " << Idx << "\n";
    Idx += Sequence.size() + 1;
  }
  auto *IntType = getMinimalTypeForRange(*std::max_element(
      SubReg2SequenceIndexMap.begin(), SubReg2SequenceIndexMap.end()));
  OS << "  };\n"
        "  static const "
     << IntType << " CompositeSequences[] = {\n";
  for (size_t I = 0, E = SubRegIndices.size(); I != E; ++I) {
    OS << "    ";
    OS << SubReg2SequenceIndexMap[I];
    if (I + 1 != E)
      OS << ",";
    OS << " // to " << SubRegIndices[I].getName() << "\n";
  }
  OS << "  };\n\n";
}

void PrinterLLVM::regInfoEmitComposeSubRegIdxLaneMask(
    std::string const &ClName,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  OS << "LaneBitmask " << ClName
     << "::composeSubRegIndexLaneMaskImpl(unsigned IdxA, "
        "LaneBitmask LaneMask) const {\n"
        "  --IdxA; assert(IdxA < "
     << SubRegIndices.size()
     << " && \"Subregister index out of bounds\");\n"
        "  LaneBitmask Result;\n"
        "  for (const MaskRolOp *Ops =\n"
        "       &LaneMaskComposeSequences[CompositeSequences[IdxA]];\n"
        "       Ops->Mask.any(); ++Ops) {\n"
        "    LaneBitmask::Type M = LaneMask.getAsInteger() & "
        "Ops->Mask.getAsInteger();\n"
        "    if (unsigned S = Ops->RotateLeft)\n"
        "      Result |= LaneBitmask((M << S) | (M >> (LaneBitmask::BitWidth - "
        "S)));\n"
        "    else\n"
        "      Result |= LaneBitmask(M);\n"
        "  }\n"
        "  return Result;\n"
        "}\n\n";
}

void PrinterLLVM::regInfoEmitComposeSubRegIdxLaneMaskRev(
    std::string const &ClName,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  OS << "LaneBitmask " << ClName
     << "::reverseComposeSubRegIndexLaneMaskImpl(unsigned IdxA, "
        " LaneBitmask LaneMask) const {\n"
        "  LaneMask &= getSubRegIndexLaneMask(IdxA);\n"
        "  --IdxA; assert(IdxA < "
     << SubRegIndices.size()
     << " && \"Subregister index out of bounds\");\n"
        "  LaneBitmask Result;\n"
        "  for (const MaskRolOp *Ops =\n"
        "       &LaneMaskComposeSequences[CompositeSequences[IdxA]];\n"
        "       Ops->Mask.any(); ++Ops) {\n"
        "    LaneBitmask::Type M = LaneMask.getAsInteger();\n"
        "    if (unsigned S = Ops->RotateLeft)\n"
        "      Result |= LaneBitmask((M >> S) | (M << (LaneBitmask::BitWidth - "
        "S)));\n"
        "    else\n"
        "      Result |= LaneBitmask(M);\n"
        "  }\n"
        "  return Result;\n"
        "}\n\n";
}

void PrinterLLVM::regInfoEmitRegBaseClassMapping(
    std::string const &ClassName,
    SmallVector<const CodeGenRegisterClass *> const BaseClasses,
    std::vector<uint8_t> const Mapping) const {
  OS << "\n// Register to base register class mapping\n\n";
  OS << "\n";
  OS << "const TargetRegisterClass *" << ClassName
     << "::getPhysRegBaseClass(MCRegister Reg)"
     << " const {\n";
  OS << "  static const TargetRegisterClass *BaseClasses["
     << (BaseClasses.size() + 1) << "] = {\n";
  OS << "    nullptr,\n";
  for (const auto *const RC : BaseClasses)
    OS << "    &" << RC->getQualifiedName() << "RegClass,\n";
  OS << "  };\n";
  OS << "  static const uint8_t Mapping[" << Mapping.size() << "] = {\n    ";
  for (const uint8_t Value : Mapping)
    OS << (unsigned)Value << ",";
  OS << "  };\n\n";
  OS << "  assert(Reg < sizeof(Mapping));\n";
  OS << "  return BaseClasses[Mapping[Reg]];\n";
  OS << "}\n";
}

} // end namespace llvm
