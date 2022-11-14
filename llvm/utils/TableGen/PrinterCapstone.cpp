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
#include "llvm/Support/FormatVariadic.h"
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
                                        bool Begin, bool Newline) const {
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

//-------------------------
// Backend: AsmWriter
//-------------------------

void PrinterCapstone::asmWriterEmitSourceFileHeader() const {
  emitSourceFileHeader("Assembly Writer Source Fragment", OS);
}

void PrinterCapstone::asmWriterEmitGetMnemonic(std::string const &TargetName,
                                           StringRef const &ClassName) const {
  OS << "/// getMnemonic - This method is automatically generated by "
        "tablegen\n"
        "/// from the instruction set description.\n"
        "std::pair<const char *, uint64_t> "
     << TargetName << ClassName << "::getMnemonic(const MCInst *MI) {\n";
}

void PrinterCapstone::asmWriterEmitAsmStrs(
    SequenceToOffsetTable<std::string> const &StrTable) const {
  StrTable.emitStringLiteralDef(OS, "  static const char AsmStrs[]");
}

void PrinterCapstone::asmWriterEmitMnemonicDecodeTable(
    unsigned const OpcodeInfoBits, unsigned BitsLeft,
    unsigned const &AsmStrBits,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions,
    std::vector<uint64_t> const &OpcodeInfo) const {
  // Emit the lookup tables in pieces to minimize wasted bytes.
  unsigned BytesNeeded = ((OpcodeInfoBits - BitsLeft) + 7) / 8;
  unsigned Table = 0, Shift = 0;
  SmallString<128> BitsString;
  raw_svector_ostream BitsOS(BitsString);
  // If the total bits is more than 32-bits we need to use a 64-bit type.
  BitsOS << "  uint" << ((BitsLeft < (OpcodeInfoBits - 32)) ? 64 : 32)
         << "_t Bits = 0;\n";
  while (BytesNeeded != 0) {
    // Figure out how big this table section needs to be, but no bigger than 4.
    unsigned TableSize = std::min(1 << Log2_32(BytesNeeded), 4);
    BytesNeeded -= TableSize;
    TableSize *= 8; // Convert to bits;
    uint64_t const Mask = (1ULL << TableSize) - 1;
    OS << "  static const uint" << TableSize << "_t OpInfo" << Table
       << "[] = {\n";
    for (unsigned I = 0, E = NumberedInstructions.size(); I != E; ++I) {
      OS << "    " << ((OpcodeInfo[I] >> Shift) & Mask) << "U,\t// "
         << NumberedInstructions[I]->TheDef->getName() << "\n";
    }
    OS << "  };\n\n";
    // Emit string to combine the individual table lookups.
    BitsOS << "  Bits |= ";
    // If the total bits is more than 32-bits we need to use a 64-bit type.
    if (BitsLeft < (OpcodeInfoBits - 32))
      BitsOS << "(uint64_t)";
    BitsOS << "OpInfo" << Table << "[MI->getOpcode()] << " << Shift << ";\n";
    // Prepare the shift for the next iteration and increment the table count.
    Shift += TableSize;
    ++Table;
  }

  OS << "  // Emit the opcode for the instruction.\n";
  OS << BitsString;

  // Return mnemonic string and bits.
  OS << "  return {AsmStrs+(Bits & " << (1 << AsmStrBits) - 1
     << ")-1, Bits};\n\n";

  OS << "}\n";
}

void PrinterCapstone::asmWriterEmitPrintInstruction(
    std::string const &TargetName,
    std::vector<std::vector<std::string>> &TableDrivenOperandPrinters,
    unsigned &BitsLeft, unsigned &AsmStrBits, StringRef const &ClassName,
    bool PassSubtarget) const {
  const unsigned OpcodeInfoBits = 64;
  // This function has some huge switch statements that causing excessive
  // compile time in LLVM profile instrumenation build. This print function
  // usually is not frequently called in compilation. Here we disable the
  // profile instrumenation for this function.
  OS << "/// printInstruction - This method is automatically generated by "
        "tablegen\n"
        "/// from the instruction set description.\n"
        "LLVM_NO_PROFILE_INSTRUMENT_FUNCTION\n"
        "void "
     << TargetName << ClassName
     << "::printInstruction(const MCInst *MI, uint64_t Address, "
     << (PassSubtarget ? "const MCSubtargetInfo &STI, " : "")
     << "raw_ostream &O) {\n";

  // Emit the initial tab character.
  OS << "  O << \"\\t\";\n\n";

  // Emit the starting string.
  OS << "  auto MnemonicInfo = getMnemonic(MI);\n\n";
  OS << "  O << MnemonicInfo.first;\n\n";

  OS << "  uint" << ((BitsLeft < (OpcodeInfoBits - 32)) ? 64 : 32)
     << "_t Bits = MnemonicInfo.second;\n"
     << "  assert(Bits != 0 && \"Cannot print this instruction.\");\n";

  // Output the table driven operand information.
  BitsLeft = OpcodeInfoBits - AsmStrBits;
  for (unsigned I = 0, E = TableDrivenOperandPrinters.size(); I != E; ++I) {
    std::vector<std::string> &Commands = TableDrivenOperandPrinters[I];

    // Compute the number of bits we need to represent these cases, this is
    // ceil(log2(numentries)).
    unsigned const NumBits = Log2_32_Ceil(Commands.size());
    assert(NumBits <= BitsLeft && "consistency error");

    // Emit code to extract this field from Bits.
    OS << "\n  // Fragment " << I << " encoded into " << NumBits << " bits for "
       << Commands.size() << " unique commands.\n";

    if (Commands.size() == 2) {
      // Emit two possibilitys with if/else.
      OS << "  if ((Bits >> " << (OpcodeInfoBits - BitsLeft) << ") & "
         << ((1 << NumBits) - 1) << ") {\n"
         << Commands[1] << "  } else {\n"
         << Commands[0] << "  }\n\n";
    } else if (Commands.size() == 1) {
      // Emit a single possibility.
      OS << Commands[0] << "\n\n";
    } else {
      OS << "  switch ((Bits >> " << (OpcodeInfoBits - BitsLeft) << ") & "
         << ((1 << NumBits) - 1) << ") {\n"
         << "  default: llvm_unreachable(\"Invalid command number.\");\n";

      // Print out all the cases.
      for (unsigned J = 0, F = Commands.size(); J != F; ++J) {
        OS << "  case " << J << ":\n";
        OS << Commands[J];
        OS << "    break;\n";
      }
      OS << "  }\n\n";
    }
    BitsLeft -= NumBits;
  }
}

void PrinterCapstone::asmWriterEmitOpCases(
    std::vector<std::pair<std::string, AsmWriterOperand>> &OpsToPrint,
    bool PassSubtarget) const {
  OS << "    case " << OpsToPrint.back().first << ":";
  AsmWriterOperand const TheOp = OpsToPrint.back().second;
  OpsToPrint.pop_back();

  // Check to see if any other operands are identical in this list, and if so,
  // emit a case label for them.
  for (unsigned I = OpsToPrint.size(); I != 0; --I)
    if (OpsToPrint[I - 1].second == TheOp) {
      OS << "\n    case " << OpsToPrint[I - 1].first << ":";
      OpsToPrint.erase(OpsToPrint.begin() + I - 1);
    }

  // Finally, emit the code.
  OS << "\n      " << TheOp.getCode(PassSubtarget);
  OS << "\n      break;\n";
}

void PrinterCapstone::asmWriterEmitInstrSwitch() const {
  OS << "  switch (MI->getOpcode()) {\n";
  OS << "  default: llvm_unreachable(\"Unexpected opcode.\");\n";
}

void PrinterCapstone::asmWriterEmitCompoundClosure(unsigned Indent, bool Newline,
                                               bool Semicolon) const {
  for (; Indent > 0; --Indent) {
    OS << " ";
  }
  OS << "}";
  if (Semicolon)
    OS << ";";
  if (Newline)
    OS << "\n";
}

void PrinterCapstone::asmWriterEmitInstruction(
    AsmWriterInst const &FirstInst,
    std::vector<AsmWriterInst> const &SimilarInsts, unsigned DifferingOperand,
    bool PassSubtarget) const {
  OS << "  case " << FirstInst.CGI->Namespace
     << "::" << FirstInst.CGI->TheDef->getName() << ":\n";
  for (const AsmWriterInst &AWI : SimilarInsts)
    OS << "  case " << AWI.CGI->Namespace << "::" << AWI.CGI->TheDef->getName()
       << ":\n";
  for (unsigned I = 0, E = FirstInst.Operands.size(); I != E; ++I) {
    if (I != DifferingOperand) {
      // If the operand is the same for all instructions, just print it.
      OS << "    " << FirstInst.Operands[I].getCode(PassSubtarget);
    } else {
      // If this is the operand that varies between all of the instructions,
      // emit a switch for just this operand now.
      OS << "    switch (MI->getOpcode()) {\n";
      OS << "    default: llvm_unreachable(\"Unexpected opcode.\");\n";
      std::vector<std::pair<std::string, AsmWriterOperand>> OpsToPrint;
      OpsToPrint.push_back(
          std::make_pair(FirstInst.CGI->Namespace.str() +
                             "::" + FirstInst.CGI->TheDef->getName().str(),
                         FirstInst.Operands[I]));

      for (const AsmWriterInst &AWI : SimilarInsts) {
        OpsToPrint.push_back(std::make_pair(
            AWI.CGI->Namespace.str() + "::" + AWI.CGI->TheDef->getName().str(),
            AWI.Operands[I]));
      }
      std::reverse(OpsToPrint.begin(), OpsToPrint.end());
      while (!OpsToPrint.empty())
        asmWriterEmitOpCases(OpsToPrint, PassSubtarget);
      OS << "    }";
    }
    OS << "\n";
  }
  OS << "    break;\n";
}

void PrinterCapstone::asmWriterEmitGetRegNameAssert(std::string const &TargetName,
                                                StringRef const &ClassName,
                                                bool HasAltNames,
                                                unsigned RegSize) const {

  OS << "\n\n/// getRegisterName - This method is automatically generated by "
        "tblgen\n"
        "/// from the register set description.  This returns the assembler "
        "name\n"
        "/// for the specified register.\n"
        "const char *"
     << TargetName << ClassName << "::";
  if (HasAltNames)
    OS << "\ngetRegisterName(unsigned RegNo, unsigned AltIdx) {\n";
  else
    OS << "getRegisterName(unsigned RegNo) {\n";
  OS << "  assert(RegNo && RegNo < " << (RegSize + 1)
     << " && \"Invalid register number!\");\n"
     << "\n";
}

void PrinterCapstone::asmWriterEmitStringLiteralDef(
    SequenceToOffsetTable<std::string> const &StringTable,
    StringRef const &AltName) const {
  StringTable.emitStringLiteralDef(OS, Twine("  static const char AsmStrs") +
                                           AltName + "[]");
}

void PrinterCapstone::asmWriterEmitRegAsmOffsets(
    unsigned RegSizes, SmallVector<std::string, 4> const &AsmNames,
    SequenceToOffsetTable<std::string> const &StringTable,
    StringRef const &AltName) const {
  OS << "  static const " << getMinimalTypeForRange(StringTable.size() - 1, 32)
     << " RegAsmOffset" << AltName << "[] = {";
  for (unsigned I = 0, E = RegSizes; I != E; ++I) {
    if ((I % 14) == 0)
      OS << "\n    ";
    OS << StringTable.get(AsmNames[I]) << ", ";
  }
  OS << "\n  };\n"
     << "\n";
}

void PrinterCapstone::asmWriterEmitAltIdxSwitch(
    bool HasAltNames, std::vector<Record *> const &AltNameIndices,
    StringRef const &Namespace) const {
  if (HasAltNames) {
    OS << "  switch(AltIdx) {\n"
       << "  default: llvm_unreachable(\"Invalid register alt name "
          "index!\");\n";
    for (const Record *R : AltNameIndices) {
      StringRef const AltName = R->getName();
      OS << "  case ";
      if (!Namespace.empty())
        OS << Namespace << "::";
      OS << AltName << ":\n";
      if (R->isValueUnset("FallbackRegAltNameIndex"))
        OS << "    assert(*(AsmStrs" << AltName << "+RegAsmOffset" << AltName
           << "[RegNo-1]) &&\n"
           << "           \"Invalid alt name index for register!\");\n";
      else {
        OS << "    if (!*(AsmStrs" << AltName << "+RegAsmOffset" << AltName
           << "[RegNo-1]))\n"
           << "      return getRegisterName(RegNo, ";
        if (!Namespace.empty())
          OS << Namespace << "::";
        OS << R->getValueAsDef("FallbackRegAltNameIndex")->getName() << ");\n";
      }
      OS << "    return AsmStrs" << AltName << "+RegAsmOffset" << AltName
         << "[RegNo-1];\n";
    }
    OS << "  }\n";
  } else {
    OS << "  assert (*(AsmStrs+RegAsmOffset[RegNo-1]) &&\n"
       << "          \"Invalid alt name index for register!\");\n"
       << "  return AsmStrs+RegAsmOffset[RegNo-1];\n";
  }
  OS << "}\n";
}

char const *PrinterCapstone::asmWriterGetPatCondKIgnore() const {
  return "AliasPatternCond::K_Ignore, 0";
}

char const *PrinterCapstone::asmWriterGetPatCondKRegClass() const {
  return "AliasPatternCond::K_RegClass, {0}::{1}RegClassID";
}

char const *PrinterCapstone::asmWriterGetPatCondKTiedReg() const {
  return "AliasPatternCond::K_TiedReg, {0}";
}

char const *PrinterCapstone::asmWriterGetPatCondKCustom() const {
  return "AliasPatternCond::K_Custom, {0}";
}

char const *PrinterCapstone::asmWriterGetPatCondKImm() const {
  return "AliasPatternCond::K_Imm, uint32_t({0})";
}

char const *PrinterCapstone::asmWriterGetPatCondKNoReg() const {
  return "AliasPatternCond::K_Reg, {0}::NoRegister";
}

char const *PrinterCapstone::asmWriterGetPatCondKReg() const {
  return "AliasPatternCond::K_Reg, {0}::{1}";
}

char const *PrinterCapstone::asmWriterGetPatCondKFeature() const {
  return "AliasPatternCond::K_{0}{1}Feature, {2}::{3}";
}

char const *PrinterCapstone::asmWriterGetPatCondKEndOrFeature() const {
  return "AliasPatternCond::K_EndOrFeatures, 0";
}

char const *PrinterCapstone::asmWriterGetPatOpcStart() const {
  return "    // {0} - {1}\n";
}

char const *PrinterCapstone::asmWriterGetCondPatStart() const {
  return "    // {0} - {1}\n";
}

std::string PrinterCapstone::asmWriterGetCond(std::string const &Cond) const {
  return formatv("    {{{0}},\n", Cond);
}

char const *PrinterCapstone::asmWriterGetPatternFormat() const {
  return "    {{{0}, {1}, {2}, {3} },\n";
}

char const *PrinterCapstone::asmWriterGetOpcodeFormat() const {
  return "    {{{0}, {1}, {2} },\n";
}

void PrinterCapstone::asmWriterEmitPrintAliasInstrHeader(
    std::string const &TargetName, StringRef const &ClassName,
    bool PassSubtarget) const {
  OS << "bool " << TargetName << ClassName << "::printAliasInstr(const MCInst"
     << " *MI, uint64_t Address, "
     << (PassSubtarget ? "const MCSubtargetInfo &STI, " : "")
     << "raw_ostream &OS) {\n";
}

void PrinterCapstone::asmWriterEmitPrintAliasInstrBodyRetFalse() const {
  OS << "  return false;\n";
  OS << "}\n\n";
}

void PrinterCapstone::asmWriterEmitDeclValid(std::string const &TargetName,
                                         StringRef const &ClassName) const {
  OS << "static bool " << TargetName << ClassName
     << "ValidateMCOperand(const MCOperand &MCOp,\n"
     << "                  const MCSubtargetInfo &STI,\n"
     << "                  unsigned PredicateIndex);\n";
}

void PrinterCapstone::asmWriterEmitPrintAliasInstrBody(
    raw_string_ostream &OpcodeO, raw_string_ostream &PatternO,
    raw_string_ostream &CondO,
    std::vector<std::pair<uint32_t, std::string>> const &AsmStrings,
    std::vector<const Record *> const &MCOpPredicates,
    std::string const &TargetName, StringRef const &ClassName,
    bool PassSubtarget) const {
  OS.indent(2) << "static const PatternsForOpcode OpToPatterns[] = {\n";
  OS << OpcodeO.str();
  OS.indent(2) << "};\n\n";
  OS.indent(2) << "static const AliasPattern Patterns[] = {\n";
  OS << PatternO.str();
  OS.indent(2) << "};\n\n";
  OS.indent(2) << "static const AliasPatternCond Conds[] = {\n";
  OS << CondO.str();
  OS.indent(2) << "};\n\n";
  OS.indent(2) << "static const char AsmStrings[] =\n";
  for (const auto &P : AsmStrings) {
    OS.indent(4) << "/* " << P.first << " */ \"" << P.second << "\\0\"\n";
  }

  OS.indent(2) << ";\n\n";

  // Assert that the opcode table is sorted. Use a static local constructor to
  // ensure that the check only happens once on first run.
  OS << "#ifndef NDEBUG\n";
  OS.indent(2) << "static struct SortCheck {\n";
  OS.indent(2) << "  SortCheck(ArrayRef<PatternsForOpcode> OpToPatterns) {\n";
  OS.indent(2) << "    assert(std::is_sorted(\n";
  OS.indent(2) << "               OpToPatterns.begin(), OpToPatterns.end(),\n";
  OS.indent(2) << "               [](const PatternsForOpcode &L, const "
                  "PatternsForOpcode &R) {\n";
  OS.indent(2) << "                 return L.Opcode < R.Opcode;\n";
  OS.indent(2) << "               }) &&\n";
  OS.indent(2) << "           \"tablegen failed to sort opcode patterns\");\n";
  OS.indent(2) << "  }\n";
  OS.indent(2) << "} sortCheckVar(OpToPatterns);\n";
  OS << "#endif\n\n";

  OS.indent(2) << "AliasMatchingData M {\n";
  OS.indent(2) << "  makeArrayRef(OpToPatterns),\n";
  OS.indent(2) << "  makeArrayRef(Patterns),\n";
  OS.indent(2) << "  makeArrayRef(Conds),\n";
  OS.indent(2) << "  StringRef(AsmStrings, array_lengthof(AsmStrings)),\n";
  if (MCOpPredicates.empty())
    OS.indent(2) << "  nullptr,\n";
  else
    OS.indent(2) << "  &" << TargetName << ClassName << "ValidateMCOperand,\n";
  OS.indent(2) << "};\n";

  OS.indent(2) << "const char *AsmString = matchAliasPatterns(MI, "
               << (PassSubtarget ? "&STI" : "nullptr") << ", M);\n";
  OS.indent(2) << "if (!AsmString) return false;\n\n";

  // Code that prints the alias, replacing the operands with the ones from the
  // MCInst.
  OS << "  unsigned I = 0;\n";
  OS << "  while (AsmString[I] != ' ' && AsmString[I] != '\\t' &&\n";
  OS << "         AsmString[I] != '$' && AsmString[I] != '\\0')\n";
  OS << "    ++I;\n";
  OS << "  OS << '\\t' << StringRef(AsmString, I);\n";

  OS << "  if (AsmString[I] != '\\0') {\n";
  OS << "    if (AsmString[I] == ' ' || AsmString[I] == '\\t') {\n";
  OS << "      OS << '\\t';\n";
  OS << "      ++I;\n";
  OS << "    }\n";
  OS << "    do {\n";
  OS << "      if (AsmString[I] == '$') {\n";
  OS << "        ++I;\n";
  OS << "        if (AsmString[I] == (char)0xff) {\n";
  OS << "          ++I;\n";
  OS << "          int OpIdx = AsmString[I++] - 1;\n";
  OS << "          int PrintMethodIdx = AsmString[I++] - 1;\n";
  OS << "          printCustomAliasOperand(MI, Address, OpIdx, "
        "PrintMethodIdx, ";
  OS << (PassSubtarget ? "STI, " : "");
  OS << "OS);\n";
  OS << "        } else\n";
  OS << "          printOperand(MI, unsigned(AsmString[I++]) - 1, ";
  OS << (PassSubtarget ? "STI, " : "");
  OS << "OS);\n";
  OS << "      } else {\n";
  OS << "        OS << AsmString[I++];\n";
  OS << "      }\n";
  OS << "    } while (AsmString[I] != '\\0');\n";
  OS << "  }\n\n";

  OS << "  return true;\n";
  OS << "}\n\n";
}

void PrinterCapstone::asmWriterEmitPrintAliasOp(
    std::string const &TargetName, StringRef const &ClassName,
    std::vector<std::pair<std::string, bool>> const &PrintMethods,
    bool PassSubtarget) const {
  OS << "void " << TargetName << ClassName << "::"
     << "printCustomAliasOperand(\n"
     << "         const MCInst *MI, uint64_t Address, unsigned OpIdx,\n"
     << "         unsigned PrintMethodIdx,\n"
     << (PassSubtarget ? "         const MCSubtargetInfo &STI,\n" : "")
     << "         raw_ostream &OS) {\n";
  if (PrintMethods.empty())
    OS << "  llvm_unreachable(\"Unknown PrintMethod kind\");\n";
  else {
    OS << "  switch (PrintMethodIdx) {\n"
       << "  default:\n"
       << "    llvm_unreachable(\"Unknown PrintMethod kind\");\n"
       << "    break;\n";

    for (unsigned I = 0; I < PrintMethods.size(); ++I) {
      OS << "  case " << I << ":\n"
         << "    " << PrintMethods[I].first << "(MI, "
         << (PrintMethods[I].second ? "Address, " : "") << "OpIdx, "
         << (PassSubtarget ? "STI, " : "") << "OS);\n"
         << "    break;\n";
    }
    OS << "  }\n";
  }
  OS << "}\n\n";
}

void PrinterCapstone::asmWriterEmitPrintMC(
    std::string const &TargetName, StringRef const &ClassName,
    std::vector<const Record *> const &MCOpPredicates) const {
  if (!MCOpPredicates.empty()) {
    OS << "static bool " << TargetName << ClassName
       << "ValidateMCOperand(const MCOperand &MCOp,\n"
       << "                  const MCSubtargetInfo &STI,\n"
       << "                  unsigned PredicateIndex) {\n"
       << "  switch (PredicateIndex) {\n"
       << "  default:\n"
       << "    llvm_unreachable(\"Unknown MCOperandPredicate kind\");\n"
       << "    break;\n";

    for (unsigned I = 0; I < MCOpPredicates.size(); ++I) {
      StringRef const MCOpPred =
          MCOpPredicates[I]->getValueAsString("MCOperandPredicate");
      OS << "  case " << I + 1 << ": {\n"
         << MCOpPred.data() << "\n"
         << "    }\n";
    }
    OS << "  }\n"
       << "}\n\n";
  }
}

} // end namespace llvm
