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
#include <regex>

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

static std::string resolveTemplateDecoder(OperandInfo const &Op) {
  unsigned const B = Op.Decoder.find_first_of("<");
  unsigned const E = Op.Decoder.find(">");
  std::string const &DecName = Op.Decoder.substr(0, B);
  std::string Args = Op.Decoder.substr(B + 1, E - B - 1);
  std::string Decoder =
      DecName + "_" + std::regex_replace(Args, std::regex("\\s*,\\s*"), "_");
  PrintNote("DEFINE_" + DecName + "(" + Decoder + ", " + Args + ")");
  return Decoder;
}

void PrinterCapstone::decoderEmitterEmitOpDecoder(raw_ostream &DecoderOS,
                                                  const OperandInfo &Op) const {
  unsigned const Indent = 4;
  DecoderOS.indent(Indent) << GuardPrefix;
  if (Op.Decoder.find("<") != std::string::npos) {
    DecoderOS << resolveTemplateDecoder(Op);
  } else {
    DecoderOS << Op.Decoder;
  }

  DecoderOS << "(MI, insn, Address, Decoder)" << GuardPostfix << " { "
            << (Op.HasCompleteDecoder ? "" : "DecodeComplete = false; ")
            << "return " << ReturnFail << "; } \\\n";
}

void PrinterCapstone::decoderEmitterEmitOpBinaryParser(
    raw_ostream &DecOS, const OperandInfo &OpInfo) const {
  unsigned const Indent = 4;
  const std::string &Decoder = (OpInfo.Decoder.find("<") != std::string::npos)
                                   ? resolveTemplateDecoder(OpInfo)
                                   : OpInfo.Decoder;

  bool const UseInsertBits = OpInfo.numFields() != 1 || OpInfo.InitValue != 0;

  if (UseInsertBits) {
    DecOS.indent(Indent) << "tmp = ";
    DecOS.write_hex(OpInfo.InitValue);
    DecOS << "; \\\n";
  }

  for (const EncodingField &EF : OpInfo) {
    DecOS.indent(Indent);
    if (UseInsertBits)
      DecOS << "tmp |= ";
    else
      DecOS << "tmp = ";
    DecOS << "fieldname(insn, " << EF.Base << ", " << EF.Width << ')';
    if (UseInsertBits)
      DecOS << " << " << EF.Offset;
    else if (EF.Offset != 0)
      DecOS << " << " << EF.Offset;
    DecOS << "; \\\n";
  }

  if (Decoder != "") {
    DecOS.indent(Indent) << GuardPrefix << Decoder
                         << "(MI, tmp, Address, Decoder)" << GuardPostfix
                         << " { "
                         << (OpInfo.HasCompleteDecoder
                                 ? ""
                                 : "DecodeComplete = false; ")
                         << "return " << ReturnFail << "; } \\\n";
  } else {
    DecOS.indent(Indent) << "MCOperand_CreateImm0(MI, tmp); \\\n";
  }
}

bool PrinterCapstone::decoderEmitterEmitPredicateMatchAux(
    const Init &Val, bool ParenIfBinOp, raw_ostream &PredOS) const {
  if (auto *D = dyn_cast<DefInit>(&Val)) {
    if (!D->getDef()->isSubClassOf("SubtargetFeature"))
      return true;
    PredOS << PredicateNamespace << "_getFeatureBits(Inst->csh->mode, "
           << PredicateNamespace << "_" << D->getAsString() << ")";
    return false;
  }
  if (auto *D = dyn_cast<DagInit>(&Val)) {
    std::string const Op = D->getOperator()->getAsString();
    if (Op == "not" && D->getNumArgs() == 1) {
      PredOS << '!';
      return decoderEmitterEmitPredicateMatchAux(*D->getArg(0), true, PredOS);
    }
    if ((Op == "any_of" || Op == "all_of") && D->getNumArgs() > 0) {
      bool const Paren =
          D->getNumArgs() > 1 && std::exchange(ParenIfBinOp, true);
      if (Paren)
        PredOS << '(';
      ListSeparator LS(Op == "any_of" ? " || " : " && ");
      for (auto *Arg : D->getArgs()) {
        PredOS << LS;
        if (decoderEmitterEmitPredicateMatchAux(*Arg, ParenIfBinOp, PredOS))
          return true;
      }
      if (Paren)
        PredOS << ')';
      return false;
    }
  }
  return true;
}

bool PrinterCapstone::decoderEmitterEmitPredicateMatch(
    raw_ostream &PredOS, const ListInit *Predicates, unsigned Opc) const {
  bool IsFirstEmission = true;
  for (unsigned I = 0; I < Predicates->size(); ++I) {
    Record *Pred = Predicates->getElementAsRecord(I);
    if (!Pred->getValue("AssemblerMatcherPredicate"))
      continue;

    if (!isa<DagInit>(Pred->getValue("AssemblerCondDag")->getValue()))
      continue;

    if (!IsFirstEmission)
      PredOS << " && ";
    if (decoderEmitterEmitPredicateMatchAux(
            *Pred->getValueAsDag("AssemblerCondDag"), Predicates->size() > 1,
            PredOS))
      PrintFatalError(Pred->getLoc(), "Invalid AssemblerCondDag!");
    IsFirstEmission = false;
  }
  return !Predicates->empty();
}

void PrinterCapstone::decoderEmitterEmitFieldFromInstruction() const {
  OS << "// Helper function for extracting fields from encoded instructions.\n"
     << "#define FieldFromInstruction(fname, InsnType) \\\n"
     << "static InsnType fname(InsnType insn, unsigned startBit, unsigned "
        "numBits) \\\n"
     << "{ \\\n"
     << "  InsnType fieldMask; \\\n"
     << "  if (numBits == sizeof(InsnType) * 8) \\\n"
     << "    fieldMask = (InsnType)(-1LL); \\\n"
     << "  else \\\n"
     << "    fieldMask = (((InsnType)1 << numBits) - 1) << startBit; \\\n"
     << "  return (insn & fieldMask) >> startBit; \\\n"
     << "}\n\n";
}

void PrinterCapstone::decoderEmitterEmitInsertBits() const { return; }

void PrinterCapstone::decoderEmitterEmitDecodeInstruction(
    bool IsVarLenInst) const {
  OS << "#define DecodeInstruction(fname, fieldname, decoder, InsnType) \\\n"
     << "static DecodeStatus fname(const uint8_t DecodeTable[], "
        "MCInst *MI, \\\n"
     << "                                      InsnType insn, uint64_t "
        "Address) { \\\n"
     << "  const uint8_t *Ptr = DecodeTable; \\\n"
     << "  uint64_t CurFieldValue = 0; \\\n"
     << "  DecodeStatus S = MCDisassembler_Success; \\\n"
     << "  while (true) { \\\n"
     << "    ptrdiff_t Loc = Ptr - DecodeTable; \\\n"
     << "    switch (*Ptr) { \\\n"
     << "    default: \\\n"
     << "      return MCDisassembler_Fail; \\\n"
     << "    case MCD_OPC_ExtractField: { \\\n"
     << "      unsigned Start = *++Ptr; \\\n"
     << "      unsigned Len = *++Ptr; \\\n"
     << "      ++Ptr; \\\n";
  if (IsVarLenInst) {
    OS << "      makeUp(insn, Start + Len); \\\n";
  }
  OS << "      CurFieldValue = fieldname(insn, Start, Len); \\\n"
     << "      break; \\\n"
     << "    } \\\n"
     << "    case MCD_OPC_FilterValue: { \\\n"
     << "      /* Decode the field value. */ \\\n"
     << "      unsigned Len; \\\n"
     << "      uint64_t Val = decodeULEB128(++Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      /* NumToSkip is a plain 24-bit integer. */ \\\n"
     << "      unsigned NumToSkip = *Ptr++; \\\n"
     << "      NumToSkip |= (*Ptr++) << 8; \\\n"
     << "      NumToSkip |= (*Ptr++) << 16; \\\n"
     << "      /* Perform the filter operation. */ \\\n"
     << "      if (Val != CurFieldValue) \\\n"
     << "        Ptr += NumToSkip; \\\n"
     << "      break; \\\n"
     << "    } \\\n"
     << "    case MCD_OPC_CheckField: { \\\n"
     << "      unsigned Start = *++Ptr; \\\n"
     << "      unsigned Len = *++Ptr; \\\n";
  if (IsVarLenInst) {
    OS << "      makeUp(insn, Start + Len); \\\n";
  }
  OS << "      uint64_t FieldValue = fieldname(insn, Start, Len); "
        "\\\n"
     << "      /* Decode the field value. */ \\\n"
     << "      unsigned PtrLen = 0; \\\n"
     << "      uint64_t ExpectedValue = decodeULEB128(++Ptr, &PtrLen); \\\n"
     << "      Ptr += PtrLen; \\\n"
     << "      /* NumToSkip is a plain 24-bit integer. */ \\\n"
     << "      unsigned NumToSkip = *Ptr++; \\\n"
     << "      NumToSkip |= (*Ptr++) << 8; \\\n"
     << "      NumToSkip |= (*Ptr++) << 16; \\\n"
     << "      /* If the actual and expected values don't match, skip. */ \\\n"
     << "      if (ExpectedValue != FieldValue) \\\n"
     << "        Ptr += NumToSkip; \\\n"
     << "      break; \\\n"
     << "    } \\\n"
     << "    case MCD_OPC_CheckPredicate: { \\\n"
     << "      unsigned Len; \\\n"
     << "      /* Decode the Predicate Index value. */ \\\n"
     << "      unsigned PIdx = decodeULEB128(++Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      /* NumToSkip is a plain 24-bit integer. */ \\\n"
     << "      unsigned NumToSkip = *Ptr++; \\\n"
     << "      NumToSkip |= (*Ptr++) << 8; \\\n"
     << "      NumToSkip |= (*Ptr++) << 16; \\\n"
     << "      /* Check the predicate. */ \\\n"
     << "      bool Pred; \\\n"
     << "      if (!(Pred = checkDecoderPredicate(PIdx))) \\\n"
     << "        Ptr += NumToSkip; \\\n"
     << "      (void)Pred; \\\n"
     << "      break; \\\n"
     << "    } \\\n"
     << "    case MCD_OPC_Decode: { \\\n"
     << "      unsigned Len; \\\n"
     << "      /* Decode the Opcode value. */ \\\n"
     << "      unsigned Opc = decodeULEB128(++Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      unsigned DecodeIdx = decodeULEB128(Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      MCInst_clear(MI); \\\n"
     << "      MCInst_setOpcode(MI, Opc); \\\n"
     << "      bool DecodeComplete; \\\n";
  if (IsVarLenInst) {
    OS << "      Len = InstrLenTable[Opc]; \\\n"
       << "      makeUp(insn, Len); \\\n";
  }
  OS << "      S = decoder(S, DecodeIdx, insn, MI, Address, "
        "&DecodeComplete); \\\n"
     << "      return S; \\\n"
     << "    } \\\n"
     << "    case MCD_OPC_TryDecode: { \\\n"
     << "      unsigned Len; \\\n"
     << "      /* Decode the Opcode value. */ \\\n"
     << "      unsigned Opc = decodeULEB128(++Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      unsigned DecodeIdx = decodeULEB128(Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      /* NumToSkip is a plain 24-bit integer. */ \\\n"
     << "      unsigned NumToSkip = *Ptr++; \\\n"
     << "      NumToSkip |= (*Ptr++) << 8; \\\n"
     << "      NumToSkip |= (*Ptr++) << 16; \\\n"
     << "      /* Perform the decode operation. */ \\\n"
     << "      MCInst_setOpcode(MI, Opc); \\\n"
     << "      bool DecodeComplete; \\\n"
     << "      S = decoder(S, DecodeIdx, insn, MI, Address, "
     << "&DecodeComplete); \\\n"
     << "      if (DecodeComplete) { \\\n"
     << "        /* Decoding complete. */ \\\n"
     << "        return S; \\\n"
     << "      } else { \\\n"
     << "        /* If the decoding was incomplete, skip. */ \\\n"
     << "        Ptr += NumToSkip; \\\n"
     << "        /* Reset decode status. This also drops a SoftFail status "
        "that could be */ \\\n"
     << "        /* set before the decode attempt. */ \\\n"
     << "        S = MCDisassembler_Success; \\\n"
     << "      } \\\n"
     << "      break; \\\n"
     << "    } \\\n"
     << "    case MCD_OPC_SoftFail: { \\\n"
     << "      /* Decode the mask values. */ \\\n"
     << "      unsigned Len; \\\n"
     << "      uint64_t PositiveMask = decodeULEB128(++Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      uint64_t NegativeMask = decodeULEB128(Ptr, &Len); \\\n"
     << "      Ptr += Len; \\\n"
     << "      bool Fail = (insn & PositiveMask) != 0 || (~insn & "
        "NegativeMask) != 0; \\\n"
     << "      if (Fail) \\\n"
     << "        S = MCDisassembler_SoftFail; \\\n"
     << "      break; \\\n"
     << "    } \\\n"
     << "    case MCD_OPC_Fail: { \\\n"
     << "      return MCDisassembler_Fail; \\\n"
     << "    } \\\n"
     << "    } \\\n"
     << "  } \\\n"
     << "  /* Bogisity detected in disassembler state machine! */ \\\n"
     << "}\n\n";
  OS << "FieldFromInstruction(fieldFromInstruction_2, uint16_t)\n"
     << "DecodeToMCInst(decodeToMCInst_2, fieldFromInstruction_2, uint16_t)\n"
     << "DecodeInstruction(decodeInstruction_2, fieldFromInstruction_2, "
        "decodeToMCInst_2, uint16_t)\n"
     << "\n"
     << "FieldFromInstruction(fieldFromInstruction_4, uint32_t)\n"
     << "DecodeToMCInst(decodeToMCInst_4, fieldFromInstruction_4, uint32_t)\n"
     << "DecodeInstruction(decodeInstruction_4, fieldFromInstruction_4, "
        "decodeToMCInst_4, uint32_t)\n";
}

void PrinterCapstone::decoderEmitterEmitTable(
    DecoderTable &Table, unsigned BitWidth, StringRef Namespace,
    std::vector<EncodingAndInst> &NumberedEncodings) const {
  unsigned Indent = 0;
  OS.indent(Indent) << "static const uint8_t DecoderTable" << Namespace
                    << BitWidth << "[] = {\n";

  Indent += 2;

  // FIXME: We may be able to use the NumToSkip values to recover
  // appropriate indentation levels.
  DecoderTable::const_iterator I = Table.begin();
  DecoderTable::const_iterator const E = Table.end();
  while (I != E) {
    assert(I < E && "incomplete decode table entry!");

    uint64_t const Pos = I - Table.begin();
    OS << "/* " << Pos << " */";
    OS.PadToColumn(12);

    switch (*I) {
    default:
      PrintFatalError("invalid decode table opcode");
    case MCD::OPC_ExtractField: {
      ++I;
      unsigned const Start = *I++;
      unsigned const Len = *I++;
      OS.indent(Indent) << "MCD_OPC_ExtractField, " << Start << ", " << Len
                        << ",  // Inst{";
      if (Len > 1)
        OS << (Start + Len - 1) << "-";
      OS << Start << "} ...\n";
      break;
    }
    case MCD::OPC_FilterValue: {
      ++I;
      OS.indent(Indent) << "MCD_OPC_FilterValue, ";
      // The filter value is ULEB128 encoded.
      while (*I >= 128)
        OS << (unsigned)*I++ << ", ";
      OS << (unsigned)*I++ << ", ";

      // 24-bit numtoskip value.
      uint8_t Byte = *I++;
      uint32_t NumToSkip = Byte;
      OS << (unsigned)Byte << ", ";
      Byte = *I++;
      OS << (unsigned)Byte << ", ";
      NumToSkip |= Byte << 8;
      Byte = *I++;
      OS << utostr(Byte) << ", ";
      NumToSkip |= Byte << 16;
      OS << "// Skip to: " << ((I - Table.begin()) + NumToSkip) << "\n";
      break;
    }
    case MCD::OPC_CheckField: {
      ++I;
      unsigned const Start = *I++;
      unsigned const Len = *I++;
      OS.indent(Indent) << "MCD_OPC_CheckField, " << Start << ", " << Len
                        << ", "; // << Val << ", " << NumToSkip << ",\n";
      // ULEB128 encoded field value.
      for (; *I >= 128; ++I)
        OS << (unsigned)*I << ", ";
      OS << (unsigned)*I++ << ", ";
      // 24-bit numtoskip value.
      uint8_t Byte = *I++;
      uint32_t NumToSkip = Byte;
      OS << (unsigned)Byte << ", ";
      Byte = *I++;
      OS << (unsigned)Byte << ", ";
      NumToSkip |= Byte << 8;
      Byte = *I++;
      OS << utostr(Byte) << ", ";
      NumToSkip |= Byte << 16;
      OS << "// Skip to: " << ((I - Table.begin()) + NumToSkip) << "\n";
      break;
    }
    case MCD::OPC_CheckPredicate: {
      ++I;
      OS.indent(Indent) << "MCD_OPC_CheckPredicate, ";
      for (; *I >= 128; ++I)
        OS << (unsigned)*I << ", ";
      OS << (unsigned)*I++ << ", ";

      // 24-bit numtoskip value.
      uint8_t Byte = *I++;
      uint32_t NumToSkip = Byte;
      OS << (unsigned)Byte << ", ";
      Byte = *I++;
      OS << (unsigned)Byte << ", ";
      NumToSkip |= Byte << 8;
      Byte = *I++;
      OS << utostr(Byte) << ", ";
      NumToSkip |= Byte << 16;
      OS << "// Skip to: " << ((I - Table.begin()) + NumToSkip) << "\n";
      break;
    }
    case MCD::OPC_Decode:
    case MCD::OPC_TryDecode: {
      bool const IsTry = *I == MCD::OPC_TryDecode;
      ++I;
      // Extract the ULEB128 encoded Opcode to a buffer.
      uint8_t Buffer[16], *P = Buffer;
      while ((*P++ = *I++) >= 128)
        assert((P - Buffer) <= (ptrdiff_t)sizeof(Buffer) &&
               "ULEB128 value too large!");
      // Decode the Opcode value.
      unsigned const Opc = decodeULEB128(Buffer);
      OS.indent(Indent) << "MCD_OPC_" << (IsTry ? "Try" : "") << "Decode, ";
      for (P = Buffer; *P >= 128; ++P)
        OS << (unsigned)*P << ", ";
      OS << (unsigned)*P << ", ";

      // Decoder index.
      for (; *I >= 128; ++I)
        OS << (unsigned)*I << ", ";
      OS << (unsigned)*I++ << ", ";

      if (!IsTry) {
        OS << "// Opcode: " << NumberedEncodings[Opc] << "\n";
        break;
      }

      // Fallthrough for OPC_TryDecode.

      // 24-bit numtoskip value.
      uint8_t Byte = *I++;
      uint32_t NumToSkip = Byte;
      OS << (unsigned)Byte << ", ";
      Byte = *I++;
      OS << (unsigned)Byte << ", ";
      NumToSkip |= Byte << 8;
      Byte = *I++;
      OS << utostr(Byte) << ", ";
      NumToSkip |= Byte << 16;

      OS << "// Opcode: " << NumberedEncodings[Opc]
         << ", skip to: " << ((I - Table.begin()) + NumToSkip) << "\n";
      break;
    }
    case MCD::OPC_SoftFail: {
      ++I;
      OS.indent(Indent) << "MCD_OPC_SoftFail";
      // Positive mask
      uint64_t Value = 0;
      unsigned Shift = 0;
      do {
        OS << ", " << (unsigned)*I;
        Value += (*I & 0x7f) << Shift;
        Shift += 7;
      } while (*I++ >= 128);
      if (Value > 127) {
        OS << " /* 0x";
        OS.write_hex(Value);
        OS << " */";
      }
      // Negative mask
      Value = 0;
      Shift = 0;
      do {
        OS << ", " << (unsigned)*I;
        Value += (*I & 0x7f) << Shift;
        Shift += 7;
      } while (*I++ >= 128);
      if (Value > 127) {
        OS << " /* 0x";
        OS.write_hex(Value);
        OS << " */";
      }
      OS << ",\n";
      break;
    }
    case MCD::OPC_Fail: {
      ++I;
      OS.indent(Indent) << "MCD_OPC_Fail,\n";
      break;
    }
    }
  }
  OS.indent(Indent) << "0\n";

  Indent -= 2;

  OS.indent(Indent) << "};\n\n";
}

void PrinterCapstone::decoderEmitterEmitInstrLenTable(
    std::vector<unsigned> &InstrLen) const {
  OS << "static const uint8_t InstrLenTable[] = {\n";
  for (unsigned const &Len : InstrLen) {
    OS << Len << ",\n";
  }
  OS << "};\n\n";
}

void PrinterCapstone::decoderEmitterEmitPredicateFunction(
    PredicateSet &Predicates, unsigned Indentation) const {
  // The predicate function is just a big switch statement based on the
  // input predicate index.
  OS.indent(Indentation) << "static bool checkDecoderPredicate(unsigned Idx"
                         << ") {\n";
  Indentation += 2;
  if (!Predicates.empty()) {
    OS.indent(Indentation) << "switch (Idx) {\n";
    OS.indent(Indentation)
        << "default: /* llvm_unreachable(\"Invalid index!\"); */\n";
    unsigned Index = 0;
    for (const auto &Predicate : Predicates) {
      OS.indent(Indentation) << "case " << Index++ << ":\n";
      OS.indent(Indentation + 2) << "return (" << Predicate << ");\n";
    }
    OS.indent(Indentation) << "}\n";
  } else {
    // No case statement to emit
    OS.indent(Indentation) << "/* llvm_unreachable(\"Invalid index!\"); */\n";
  }
  Indentation -= 2;
  OS.indent(Indentation) << "}\n\n";
}

void PrinterCapstone::decoderEmitterEmitDecoderFunction(
    DecoderSet &Decoders, unsigned Indentation) const {
  // The decoder function is just a big switch statement based on the
  // input decoder index.
  OS.indent(Indentation)
      << "#define DecodeToMCInst(fname, fieldname, InsnType) \\\n"
      << "static DecodeStatus fname(DecodeStatus S, unsigned Idx, InsnType "
         "insn, MCInst *MI, \\\n"
      << "		uint64_t Address, bool *Decoder) \\\n"
      << "{ \\\n";
  Indentation += 2;
  OS.indent(Indentation) << "InsnType tmp; \\\n";
  OS.indent(Indentation) << "switch (Idx) { \\\n";
  OS.indent(Indentation)
      << "default: /* llvm_unreachable(\"Invalid index!\"); */ \\\n";
  unsigned Index = 0;
  for (const auto &Decoder : Decoders) {
    OS.indent(Indentation) << "case " << Index++ << ": \\\n";
    OS << Decoder;
    OS.indent(Indentation + 2) << "return S; \\\n";
  }
  OS.indent(Indentation) << "} \\\n";
  Indentation -= 2;
  OS.indent(Indentation) << "}\n\n";
}

void PrinterCapstone::decoderEmitterEmitIncludes() const {
  OS << "#include \"../../MCInst.h\"\n"
     << "#include \"../../LEB128.h\"\n\n";
}

void PrinterCapstone::decoderEmitterEmitSourceFileHeader() const {
  OS << "/* Capstone Disassembly Engine, http://www.capstone-engine.org */\n";
  OS << "/* By Nguyen Anh Quynh <aquynh@gmail.com>, 2013-2022, */\n";
  OS << "/*    Rot127 <unisono@quyllur.org> 2022 */\n";
  OS << "/* Automatically generated file by the LLVM TableGen Disassembler "
        "Backend. */\n";
  OS << "/* Do not edit. */\n\n";
}

} // end namespace llvm
