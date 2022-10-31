//===------------- PrinterCapstone.cpp - Printer Capstone -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Emits the generated decoder C code for the Capstone.
//
//===----------------------------------------------------------------------===//

#include "Printer.h"
#include "llvm/TableGen/TableGenBackend.h"

namespace llvm {

/// Prints `namespace <name> {` and `} // end namespace <name>` to the output
/// stream. If Name == "" it emits an anonymous namespace.
void PrinterCapstone::emitNamespace(std::string const &Name, bool Begin,
                                    std::string const &Comment) const {
  return;
}

/// Prints
/// ```
/// #ifdef <Name>
/// #undef <Name>
/// ```
/// and
/// `#endif // <Name>`
/// Used to control inclusion of a code block via a macro definition.
void PrinterCapstone::emitIncludeToggle(std::string const &Name,
                                        bool Begin) const {
  if (Name == "GET_REGINFO_TARGET_DESC" ||
      Name == "GET_REGINFO_HEADER") { 
    return;
  }
  if (Begin) {
    OS << "\n#ifdef " << Name << "\n";
    OS << "#undef " << Name << "\n\n";
  } else {
    OS << "#endif // " << Name << "\n\n";
  }
}

void PrinterCapstone::regInfoEmitSourceFileHeader(
    std::string const &Desc) const {
  static unsigned Count = 0;
  if (Count > 1) {
    // Only emit it once at the beginning.
    return;
  }
  OS << "/* Capstone Disassembly Engine, http://www.capstone-engine.org */\n";
  OS << "/* By Nguyen Anh Quynh <aquynh@gmail.com>, 2013-2022, */\n";
  OS << "/*    Rot127 <unisono@quyllur.org> 2022 */\n";
  OS << "/* Automatically generated file by the LLVM TableGen Disassembler "
        "Backend. */\n";
  OS << "/* Do not edit. */\n\n";
  ++Count;
}

// runEnums - Print out enum values for all of the registers.
void PrinterCapstone::regInfoEmitEnums(CodeGenTarget const &Target,
                                       CodeGenRegBank const &Bank) const {
  const auto &Registers = Bank.getRegisters();

  // Register enums are stored as uint16_t in the tables. Make sure we'll fit.
  assert(Registers.size() <= 0xffff && "Too many regs to fit in tables");

  emitIncludeToggle("GET_REGINFO_ENUM", true);
  std::string const TargetName = Target.getName().str();

  OS << "enum {\n  " << TargetName << "_NoRegister,\n";

  for (const auto &Reg : Registers)
    OS << "  " << TargetName << "_" << Reg.getName() << " = " << Reg.EnumValue << ",\n";
  assert(Registers.size() == Registers.back().EnumValue &&
         "Register enum value mismatch!");
  OS << "  NUM_TARGET_REGS // " << Registers.size() + 1 << "\n";
  OS << "};\n";

  const auto &RegisterClasses = Bank.getRegClasses();
  if (!RegisterClasses.empty()) {

    // RegisterClass enums are stored as uint16_t in the tables.
    assert(RegisterClasses.size() <= 0xffff &&
           "Too many register classes to fit in tables");

    OS << "\n// Register classes\n\n";
    OS << "enum {\n";
    for (const auto &RC : RegisterClasses)
      OS << "  " << TargetName << "_" << RC.getName() << "RegClassID"
         << " = " << RC.EnumValue << ",\n";
    OS << "\n};\n";
  }

  const std::vector<Record *> &RegAltNameIndices =
      Target.getRegAltNameIndices();
  // If the only definition is the default NoRegAltName, we don't need to
  // emit anything.
  if (RegAltNameIndices.size() > 1) {
    OS << "\n// Register alternate name indices\n\n";
    OS << "enum {\n";
    for (unsigned I = 0, E = RegAltNameIndices.size(); I != E; ++I)
      OS << "  " << TargetName << "_" << RegAltNameIndices[I]->getName() << ",\t// " << I << "\n";
    OS << "  NUM_TARGET_REG_ALT_NAMES = " << RegAltNameIndices.size() << "\n";
    OS << "};\n";
  }

  auto &SubRegIndices = Bank.getSubRegIndices();
  if (!SubRegIndices.empty()) {
    OS << "\n// Subregister indices\n\n";
    std::string const Namespace = SubRegIndices.front().getNamespace();
    OS << "enum {\n  " << TargetName << "_NoSubRegister,\n";
    unsigned I = 0;
    for (const auto &Idx : SubRegIndices)
      OS << "  " << TargetName << "_" << Idx.getName() << ",\t// " << ++I << "\n";
    OS << "  " << TargetName << "_NUM_TARGET_SUBREGS\n};\n";
  }
  emitIncludeToggle("GET_REGINFO_ENUM", false);
}

void PrinterCapstone::regInfoEmitRegDiffLists(
    std::string const TargetName,
    SequenceToOffsetTable<DiffVec> const &DiffSeqs) const {
  OS << "static const MCPhysReg " << TargetName << "RegDiffLists[] = {\n";
  DiffSeqs.emit(OS, [](raw_ostream &OS, uint16_t Val) { OS << Val; });
  OS << "};\n\n";
}

void PrinterCapstone::regInfoEmitLaneMaskLists(
    std::string const TargetName,
    SequenceToOffsetTable<MaskVec> const &LaneMaskSeqs) const {
  return;
}

void PrinterCapstone::regInfoEmitSubRegIdxLists(
    std::string const TargetName,
    SequenceToOffsetTable<SubRegIdxVec, deref<std::less<>>> const
        &SubRegIdxSeqs) const {
  OS << "static const uint16_t " << TargetName << "SubRegIdxLists[] = {\n";
  SubRegIdxSeqs.emit(OS, [](raw_ostream &OS, const CodeGenSubRegIndex *Idx) {
    OS << Idx->EnumValue;
  });
  OS << "};\n\n";
}

void PrinterCapstone::regInfoEmitSubRegIdxSizes(
    std::string const TargetName,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  return;
}

void PrinterCapstone::regInfoEmitSubRegStrTable(
    std::string const TargetName,
    SequenceToOffsetTable<std::string> const &RegStrings) const {
  OS << "static const MCRegisterDesc " << TargetName
     << "RegDesc[] = { // Descriptors\n";
  OS << "  { " << RegStrings.get("") << ", 0, 0, 0, 0, 0 },\n";
}

void PrinterCapstone::regInfoEmitRegDesc(
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

void PrinterCapstone::regInfoEmitRegUnitRoots(
    std::string const TargetName, CodeGenRegBank const &RegBank) const {
  return;
}


static std::string getQualifiedNameCCS(CodeGenRegisterClass const &RC) {
  if (RC.Namespace.empty())
    return RC.getName();
  return (RC.Namespace + "_" + RC.getName()).str();
}

static std::string getQualifiedNameCCS(const Record *R) {
  std::string Namespace;
  if (R->getValue("Namespace"))
    Namespace = std::string(R->getValueAsString("Namespace"));
  if (Namespace.empty())
    return std::string(R->getName());
  return Namespace + "_" + R->getName().str();
}

void PrinterCapstone::regInfoEmitRegClasses(
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
         << "  static const MCPhysReg " << Name << "[] = {\n    ";
      for (Record *Reg : Order) {
        OS << getQualifiedNameCCS(Reg) << ", ";
      }
      OS << "\n  };\n\n";

      OS << "  // " << Name << " Bit set.\n"
         << "  static const uint8_t " << Name << "Bits[] = {\n    ";
      PrinterBitVectorEmitter BVE;
      for (Record *Reg : Order) {
        BVE.add(Target.getRegBank().getReg(Reg)->EnumValue);
      }
      BVE.print(OS);
      OS << "\n  };\n\n";
    }
  }
}

void PrinterCapstone::regInfoEmitStrLiteralRegClasses(
    std::string const TargetName,
    SequenceToOffsetTable<std::string> const &RegClassStrings) const {
  return;
}

void PrinterCapstone::regInfoEmitMCRegClassesTable(
    std::string const TargetName,
    std::list<CodeGenRegisterClass> const &RegClasses,
    SequenceToOffsetTable<std::string> &RegClassStrings) const {
  OS << "static const MCRegisterClass " << TargetName
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
       << ", " << RCBitsSize << ", " << getQualifiedNameCCS(RC) + "RegClassID"
       << ", " << RegSize << ", " << RC.CopyCost << ", "
       << (RC.Allocatable ? "true" : "false") << " },\n";
  }

  OS << "};\n\n";
}

void PrinterCapstone::regInfoEmitRegEncodingTable(
    std::string const TargetName,
    std::deque<CodeGenRegister> const &Regs) const {
  return;
}

void PrinterCapstone::regInfoEmitMCRegInfoInit(
    std::string const TargetName, CodeGenRegBank const &RegBank,
    std::deque<CodeGenRegister> const &Regs,
    std::list<CodeGenRegisterClass> const &RegClasses,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  return;
}

void PrinterCapstone::regInfoEmitInfoDwarfRegsRev(
    StringRef const &Namespace, DwarfRegNumsVecTy &DwarfRegNums,
    unsigned MaxLength, bool IsCtor) const {
  return;
}

void PrinterCapstone::regInfoEmitInfoDwarfRegs(StringRef const &Namespace,
                                               DwarfRegNumsVecTy &DwarfRegNums,
                                               unsigned MaxLength,
                                               bool IsCtor) const {
  return;
}

void PrinterCapstone::regInfoEmitInfoRegMapping(StringRef const &Namespace,
                                                unsigned MaxLength,
                                                bool IsCtor) const {
  return;
}

void PrinterCapstone::regInfoEmitHeaderIncludes() const {
  return;
}

void PrinterCapstone::regInfoEmitHeaderExternRegClasses(
    std::list<CodeGenRegisterClass> const &RegClasses) const {
  return;
}

void PrinterCapstone::regInfoEmitHeaderDecl(std::string const &TargetName,
                                            std::string const &ClassName,
                                            bool SubRegsPresent,
                                            bool DeclareGetPhysRegBaseClass) const {
  return;
}

void PrinterCapstone::regInfoEmitExternRegClassesArr(
    std::string const &TargetName) const {
  return;
}

void PrinterCapstone::regInfoEmitVTSeqs(
    SequenceToOffsetTable<std::vector<MVT::SimpleValueType>> const &VTSeqs)
    const {
  return;
}

void PrinterCapstone::regInfoEmitSubRegIdxTable(
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  return;
}

void PrinterCapstone::regInfoEmitRegClassInfoTable(
    std::list<CodeGenRegisterClass> const &RegClasses,
    SequenceToOffsetTable<std::vector<MVT::SimpleValueType>> const &VTSeqs,
    CodeGenHwModes const &CGH, unsigned NumModes) const {
  return;
}

void PrinterCapstone::regInfoEmitSubClassMaskTable(
    std::list<CodeGenRegisterClass> const &RegClasses,
    SmallVector<IdxList, 8> &SuperRegIdxLists,
    SequenceToOffsetTable<IdxList, deref<std::less<>>> &SuperRegIdxSeqs,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices,
    BitVector &MaskBV) const {
  return;
}

void PrinterCapstone::regInfoEmitSuperRegIdxSeqsTable(
    SequenceToOffsetTable<IdxList, deref<std::less<>>> const &SuperRegIdxSeqs)
    const {
  return;
}

void PrinterCapstone::regInfoEmitSuperClassesTable(
    std::list<CodeGenRegisterClass> const &RegClasses) const {
  return;
}

void PrinterCapstone::regInfoEmitRegClassMethods(
    std::list<CodeGenRegisterClass> const &RegClasses,
    std::string const &TargetName) const {
  return;
}

void PrinterCapstone::regInfomitRegClassInstances(
    std::list<CodeGenRegisterClass> const &RegClasses,
    SequenceToOffsetTable<IdxList, deref<std::less<>>> const &SuperRegIdxSeqs,
    SmallVector<IdxList, 8> const &SuperRegIdxLists,
    std::string const &TargetName) const {
  return;
}

void PrinterCapstone::regInfoEmitRegClassTable(
    std::list<CodeGenRegisterClass> const &RegClasses) const {
  return;
}

void PrinterCapstone::regInfoEmitCostPerUseTable(
    std::vector<unsigned> const &AllRegCostPerUse, unsigned NumRegCosts) const {
  return;
}

void PrinterCapstone::regInfoEmitInAllocatableClassTable(
    llvm::BitVector const &InAllocClass) const {
  return;
}

void PrinterCapstone::regInfoEmitRegExtraDesc(std::string const &TargetName,
                                              unsigned NumRegCosts) const {
  return;
}

void PrinterCapstone::regInfoEmitSubClassSubRegGetter(
    std::string const &ClassName, unsigned SubRegIndicesSize,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices,
    std::list<CodeGenRegisterClass> const &RegClasses,
    CodeGenRegBank &RegBank) const {
  return;
}

void PrinterCapstone::regInfoEmitRegClassWeight(
    CodeGenRegBank const &RegBank, std::string const &ClassName) const {
  return;
}

void PrinterCapstone::regInfoEmitRegUnitWeight(
    CodeGenRegBank const &RegBank, std::string const &ClassName,
    bool RegUnitsHaveUnitWeight) const {
  return;
}

void PrinterCapstone::regInfoEmitGetNumRegPressureSets(
    std::string const &ClassName, unsigned NumSets) const {
  return;
}

void PrinterCapstone::regInfoEmitGetRegPressureTables(
    CodeGenRegBank const &RegBank, std::string const &ClassName,
    unsigned NumSets) const {
  return;
}

void PrinterCapstone::regInfoEmitRCSetsTable(
    std::string const &ClassName, unsigned NumRCs,
    SequenceToOffsetTable<std::vector<int>> const &PSetsSeqs,
    std::vector<std::vector<int>> const &PSets) const {
  return;
}

void PrinterCapstone::regInfoEmitGetRegUnitPressureSets(
    SequenceToOffsetTable<std::vector<int>> const &PSetsSeqs,
    CodeGenRegBank const &RegBank, std::string const &ClassName,
    std::vector<std::vector<int>> const &PSets) const {
  return;
}

void PrinterCapstone::regInfoEmitExternTableDecl(
    std::string const &TargetName) const {
  return;
}

void PrinterCapstone::regInfoEmitRegClassInit(
    std::string const &TargetName, std::string const &ClassName,
    CodeGenRegBank const &RegBank,
    std::list<CodeGenRegisterClass> const &RegClasses,
    std::deque<CodeGenRegister> const &Regs, unsigned SubRegIndicesSize) const {
  return;
}

void PrinterCapstone::regInfoEmitSaveListTable(
    Record const *CSRSet, SetTheory::RecVec const *Regs) const {
  return;
}

void PrinterCapstone::regInfoEmitRegMaskTable(std::string const &CSRSetName,
                                              BitVector &Covered) const {
  return;
}

void PrinterCapstone::regInfoEmitGetRegMasks(
    std::vector<Record *> const &CSRSets, std::string const &ClassName) const {
  return;
}

void PrinterCapstone::regInfoEmitGPRCheck(
    std::string const &ClassName,
    std::list<CodeGenRegisterCategory> const &RegCategories) const {
  return;
}
void PrinterCapstone::regInfoEmitFixedRegCheck(
    std::string const &ClassName,
    std::list<CodeGenRegisterCategory> const &RegCategories) const {
  return;
}

void PrinterCapstone::regInfoEmitArgRegCheck(
    std::string const &ClassName,
    std::list<CodeGenRegisterCategory> const &RegCategories) const {
  return;
}

void PrinterCapstone::regInfoEmitGetRegMaskNames(
    std::vector<Record *> const &CSRSets, std::string const &ClassName) const {
  return;
}

void PrinterCapstone::regInfoEmitGetFrameLowering(
    std::string const &TargetName) const {
  return;
}

void PrinterCapstone::regInfoEmitComposeSubRegIndicesImplHead(
    std::string const &ClName) const {
  return;
}

void PrinterCapstone::regInfoEmitComposeSubRegIndicesImplBody(
    SmallVector<SmallVector<CodeGenSubRegIndex *, 4>, 4> const &Rows,
    unsigned SubRegIndicesSize, SmallVector<unsigned, 4> const &RowMap) const {
  return;
}

void PrinterCapstone::regInfoEmitLaneMaskComposeSeq(
    SmallVector<SmallVector<MaskRolPair, 1>, 4> const &Sequences,
    SmallVector<unsigned, 4> const &SubReg2SequenceIndexMap,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  return;
}

void PrinterCapstone::regInfoEmitComposeSubRegIdxLaneMask(
    std::string const &ClName,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  return;
}

void PrinterCapstone::regInfoEmitComposeSubRegIdxLaneMaskRev(
    std::string const &ClName,
    std::deque<CodeGenSubRegIndex> const &SubRegIndices) const {
  return;
}

} // end namespace llvm
