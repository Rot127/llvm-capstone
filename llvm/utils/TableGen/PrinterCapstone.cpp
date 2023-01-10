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
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/FormatVariadic.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/TableGen/TableGenBackend.h"
#include <regex>

static void emitDefaultSourceFileHeader(formatted_raw_ostream &OS) {
  OS << "/* Capstone Disassembly Engine, http://www.capstone-engine.org */\n";
  OS << "/* By Nguyen Anh Quynh <aquynh@gmail.com>, 2013-2022, */\n";
  OS << "/*    Rot127 <unisono@quyllur.org> 2022 */\n";
  OS << "/* Automatically generated file by the LLVM TableGen Disassembler "
        "Backend. */\n";
  OS << "/* Do not edit. */\n\n";
}

namespace llvm {

/// Prints `namespace <name> {` and `} // end namespace <name>` to the output
/// stream. If Name == "" it emits an anonymous namespace.
void PrinterCapstone::emitNamespace(std::string const &Name, bool Begin,
                                    std::string const &Comment = "") const {
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
void PrinterCapstone::emitIncludeToggle(std::string const &Name, bool Begin,
                                        bool Newline, bool UndefAtEnd) const {
  if (Name == "GET_REGINFO_TARGET_DESC" || Name == "GET_REGINFO_HEADER" ||
      Name == "GET_MNEMONIC_CHECKER" || Name == "GET_MNEMONIC_SPELL_CHECKER" ||
      Name == "GET_MATCHER_IMPLEMENTATION" ||
      Name == "GET_SUBTARGET_FEATURE_NAME" || Name == "GET_REGISTER_MATCHER" ||
      Name == "GET_OPERAND_DIAGNOSTIC_TYPES" ||
      Name == "GET_ASSEMBLER_HEADER") {
    return;
  }
  if (Begin) {
    OS << "#ifdef " << Name << "\n";
    if (!UndefAtEnd)
      OS << "#undef " << Name << "\n\n";
  } else {
    if (UndefAtEnd)
      OS << "#undef " << Name << "\n";
    OS << "#endif // " << Name << (Newline ? "\n\n" : "\n");
  }
}

void PrinterCapstone::regInfoEmitSourceFileHeader(
    std::string const &Desc) const {
  static unsigned Count = 0;
  if (Count > 1) {
    // Only emit it once at the beginning.
    return;
  }
  emitDefaultSourceFileHeader(OS);
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
    OS << "  " << TargetName << "_" << Reg.getName() << " = " << Reg.EnumValue
       << ",\n";
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
      OS << "  " << TargetName << "_" << RegAltNameIndices[I]->getName()
         << ",\t// " << I << "\n";
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
      OS << "  " << TargetName << "_" << Idx.getName() << ",\t// " << ++I
         << "\n";
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

void PrinterCapstone::regInfoEmitHeaderIncludes() const { return; }

void PrinterCapstone::regInfoEmitHeaderExternRegClasses(
    std::list<CodeGenRegisterClass> const &RegClasses) const {
  return;
}

void PrinterCapstone::regInfoEmitHeaderDecl(
    std::string const &TargetName, std::string const &ClassName,
    bool SubRegsPresent, bool DeclareGetPhysRegBaseClass) const {
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
  emitDefaultSourceFileHeader(OS);
}

//-------------------------
// Backend: AsmWriter
//-------------------------

void PrinterCapstone::asmWriterEmitSourceFileHeader() const {
  emitDefaultSourceFileHeader(OS);
  OS << "#include <capstone/platform.h>"
     << "#include <assert.h>";
}

void PrinterCapstone::asmWriterEmitGetMnemonic(
    std::string const &TargetName, StringRef const &ClassName) const {
  OS << "/// getMnemonic - This method is automatically generated by "
        "tablegen\n"
        "/// from the instruction set description.\n"
        "std::pair<const char *, uint64_t> getMnemonic(MCInst *MI) {\n";
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
        "static void "
     << "printInstruction(const MCInst *MI, uint64_t Address, "
     << (PassSubtarget ? "const MCSubtargetInfo &STI, " : "")
     << "SStream *O) {\n";

  // Emit the initial tab character.
  OS << "   SStream_concat0(O, \"\\t\";\n\n)";

  // Emit the starting string.
  OS << "  auto MnemonicInfo = getMnemonic(MI);\n\n";
  OS << "  SStream_concat0(O, MnemonicInfo.first);\n\n";

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

void PrinterCapstone::asmWriterEmitCompoundClosure(unsigned Indent,
                                                   bool Newline,
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

void PrinterCapstone::asmWriterEmitGetRegNameAssert(
    std::string const &TargetName, StringRef const &ClassName, bool HasAltNames,
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
  OS << "static bool printAliasInstr(const MCInst"
     << " *MI, uint64_t Address, "
     << (PassSubtarget ? "const MCSubtargetInfo &STI, " : "")
     << "SStream *OS) {\n";
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
  OS << "static void printCustomAliasOperand(\n"
     << "         const MCInst *MI, uint64_t Address, unsigned OpIdx,\n"
     << "         unsigned PrintMethodIdx,\n"
     << (PassSubtarget ? "         const MCSubtargetInfo &STI,\n" : "")
     << "         SStream *OS) {\n";
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

//-------------------------
// Backend: Subtarget
//-------------------------

void PrinterCapstone::subtargetEmitSourceFileHeader() const {
  emitSourceFileHeader("Subtarget Enumeration Source Fragment", OS);
}

void PrinterCapstone::subtargetEmitFeatureEnum(
    DenseMap<Record *, unsigned> &FeatureMap,
    std::vector<Record *> const &DefList, unsigned N) const {
  // Open enumeration.
  OS << "enum {\n";

  // For each record
  for (unsigned I = 0; I < N; ++I) {
    // Next record
    Record *Def = DefList[I];

    // Get and emit name
    OS << "  " << Def->getName() << " = " << I << ",\n";

    // Save the index for this feature.
    FeatureMap[Def] = I;
  }

  OS << "  "
     << "NumSubtargetFeatures = " << N << "\n";

  // Close enumeration and namespace
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitGetSTIMacro(
    StringRef const &Value, StringRef const &Attribute) const {
  // Some features default to true, with values set to false if enabled.
  const char *Default = Value == "false" ? "true" : "false";

  // Define the getter with lowercased first char: xxxYyy() { return XxxYyy; }
  const std::string Getter =
      Attribute.substr(0, 1).lower() + Attribute.substr(1).str();

  OS << "GET_SUBTARGETINFO_MACRO(" << Attribute << ", " << Default << ", "
     << Getter << ")\n";
}

void PrinterCapstone::subtargetEmitHwModes(CodeGenHwModes const &CGH,
                                           std::string const &ClassName) const {
  OS << "unsigned " << ClassName << "::getHwMode() const {\n";
  for (unsigned M = 1, NumModes = CGH.getNumModeIds(); M != NumModes; ++M) {
    const HwMode &HM = CGH.getMode(M);
    OS << "  if (checkFeatures(\"" << HM.Features << "\")) return " << M
       << ";\n";
  }
  OS << "  return 0;\n}\n";
}

void PrinterCapstone::subtargetEmitFeatureKVHeader(
    std::string const &Target) const {
  // Begin feature table
  OS << "// Sorted (by key) array of values for CPU features.\n"
     << "extern const llvm::SubtargetFeatureKV " << Target
     << "FeatureKV[] = {\n";
}

void PrinterCapstone::subtargetEmitFeatureKVPartI(
    std::string const &Target, StringRef const &CommandLineName,
    StringRef const &Name, StringRef const &Desc) const {
  // Emit as { "feature", "description", { featureEnum }, { i1 , i2 , ... , in }
  OS << "  { "
     << "\"" << CommandLineName << "\", "
     << "\"" << Desc << "\", " << Target << "::" << Name << ", ";
}

void PrinterCapstone::subtargetEmitFeatureKVPartII() const { OS << " },\n"; }

void PrinterCapstone::subtargetEmitPrintFeatureMask(
    std::array<uint64_t, MAX_SUBTARGET_WORDS> const &Mask) const {
  OS << "{ { { ";
  for (unsigned I = 0; I != Mask.size(); ++I) {
    OS << "0x";
    OS.write_hex(Mask[I]);
    OS << "ULL, ";
  }
  OS << "} } }";
}

void PrinterCapstone::subtargetEmitFeatureKVEnd() const { OS << "};\n"; }

void PrinterCapstone::subtargetEmitCPUKVHeader(
    std::string const &Target) const {
  OS << "// Sorted (by key) array of values for CPU subtype.\n"
     << "extern const llvm::SubtargetSubTypeKV " << Target
     << "SubTypeKV[] = {\n";
}

void PrinterCapstone::subtargetEmitCPUKVEnd() const { OS << "};\n"; }

void PrinterCapstone::subtargetEmitCPUKVPartI(StringRef const &Name) const {
  // Emit as { "cpu", "description", 0, { f1 , f2 , ... fn } },
  OS << " { "
     << "\"" << Name << "\", ";
}

void PrinterCapstone::subtargetEmitCPUKVPartII() const { OS << ", "; }

void PrinterCapstone::subtargetEmitCPUKVPartIII(
    std::string const &ProcModelName) const {
  OS << ", &" << ProcModelName << " },\n";
}

void PrinterCapstone::subtargetEmitDBGMacrosBegin() const {
  OS << "#ifdef DBGFIELD\n"
     << "#error \"<target>GenSubtargetInfo.inc requires a DBGFIELD macro\"\n"
     << "#endif\n"
     << "#if !defined(NDEBUG) || defined(LLVM_ENABLE_DUMP)\n"
     << "#define DBGFIELD(x) x,\n"
     << "#else\n"
     << "#define DBGFIELD(x)\n"
     << "#endif\n";
}

void PrinterCapstone::subtargetEmitDBGMacrosEnd() const {
  OS << "\n#undef DBGFIELD\n";
}

void PrinterCapstone::subtargetEmitFunctionalItinaryUnits(
    CodeGenSchedModels const &SchedModels) const {

  // Multiple processor models may share an itinerary record. Emit it once.
  SmallPtrSet<Record *, 8> ItinsDefSet;

  // Emit functional units for all the itineraries.
  for (const CodeGenProcModel &ProcModel : SchedModels.procModels()) {

    if (!ItinsDefSet.insert(ProcModel.ItinsDef).second)
      continue;

    RecVec FUs = ProcModel.ItinsDef->getValueAsListOfDefs("FU");
    if (FUs.empty())
      continue;

    StringRef const Name = ProcModel.ItinsDef->getName();
    OS << "\n// Functional units for \"" << Name << "\"\n"
       << "namespace " << Name << "FU {\n";

    for (unsigned J = 0, FUN = FUs.size(); J < FUN; ++J)
      OS << "  const InstrStage::FuncUnits " << FUs[J]->getName()
         << " = 1ULL << " << J << ";\n";

    OS << "} // end namespace " << Name << "FU\n";

    RecVec BPs = ProcModel.ItinsDef->getValueAsListOfDefs("BP");
    if (!BPs.empty()) {
      OS << "\n// Pipeline forwarding paths for itineraries \"" << Name
         << "\"\n"
         << "namespace " << Name << "Bypass {\n";

      OS << "  const unsigned NoBypass = 0;\n";
      for (unsigned J = 0, BPN = BPs.size(); J < BPN; ++J)
        OS << "  const unsigned " << BPs[J]->getName() << " = 1 << " << J
           << ";\n";

      OS << "} // end namespace " << Name << "Bypass\n";
    }
  }
}

std::string const PrinterCapstone::subtargetGetBeginStageTable(
    std::string const &TargetName) const {
  return "\nextern const llvm::InstrStage " + TargetName + "Stages[] = {\n" +
         "  { 0, 0, 0, llvm::InstrStage::Required }, // No itinerary\n";
}

std::string const PrinterCapstone::subtargetGetBeginOperandCycleTable(
    std::string const &TargetName) const {
  return "extern const unsigned " + TargetName + "OperandCycles[] = {\n" +
         "  0, // No itinerary\n";
}

std::string const PrinterCapstone::subtargetGetBeginBypassTable(
    std::string const &TargetName) const {
  return "extern const unsigned " + TargetName + "ForwardingPaths[] = {\n" +
         " 0, // No itinerary\n";
}

std::string const PrinterCapstone::subtargetGetEndStageTable() const {
  return "  { 0, 0, 0, llvm::InstrStage::Required } // End stages\n};\n";
}

std::string const PrinterCapstone::subtargetGetEndOperandCycleTable() const {
  return "  0 // End operand cycles\n};\n";
}

std::string const PrinterCapstone::subtargetGetEndBypassTable() const {
  return " 0 // End bypass tables\n};\n";
}

// subtargetFormItineraryStageString - Compose a string containing the stage
// data initialization for the specified itinerary.  N is the number
// of stages.
void PrinterCapstone::subtargetFormItineraryStageString(
    std::string const &Name, Record *ItinData, std::string &ItinString,
    unsigned &NStages) const {
  // Get states list
  RecVec StageList = ItinData->getValueAsListOfDefs("Stages");

  // For each stage
  unsigned const N = NStages = StageList.size();
  for (unsigned I = 0; I < N;) {
    // Next stage
    const Record *Stage = StageList[I];

    // Form string as ,{ cycles, u1 | u2 | ... | un, timeinc, kind }
    int const Cycles = Stage->getValueAsInt("Cycles");
    ItinString += "  { " + itostr(Cycles) + ", ";

    // Get unit list
    RecVec UnitList = Stage->getValueAsListOfDefs("Units");

    // For each unit
    for (unsigned J = 0, M = UnitList.size(); J < M;) {
      // Add name and bitwise or
      ItinString += Name + "FU::" + UnitList[J]->getName().str();
      if (++J < M)
        ItinString += " | ";
    }

    int const TimeInc = Stage->getValueAsInt("TimeInc");
    ItinString += ", " + itostr(TimeInc);

    int const Kind = Stage->getValueAsInt("Kind");
    ItinString += ", (llvm::InstrStage::ReservationKinds)" + itostr(Kind);

    // Close off stage
    ItinString += " }";
    if (++I < N)
      ItinString += ", ";
  }
}

// FormItineraryOperandCycleString - Compose a string containing the
// operand cycle initialization for the specified itinerary.  N is the
// number of operands that has cycles specified.
void PrinterCapstone::subtargetFormItineraryOperandCycleString(
    Record *ItinData, std::string &ItinString, unsigned &NOperandCycles) const {
  // Get operand cycle list
  std::vector<int64_t> OperandCycleList =
      ItinData->getValueAsListOfInts("OperandCycles");

  // For each operand cycle
  NOperandCycles = OperandCycleList.size();
  ListSeparator LS;
  for (int OCycle : OperandCycleList) {
    // Next operand cycle
    ItinString += LS;
    ItinString += "  " + itostr(OCycle);
  }
}

void PrinterCapstone::subtargetFormItineraryBypassString(
    const std::string &Name, Record *ItinData, std::string &ItinString,
    unsigned NOperandCycles) const {
  RecVec BypassList = ItinData->getValueAsListOfDefs("Bypasses");
  unsigned const N = BypassList.size();
  unsigned I = 0;
  ListSeparator LS;
  for (; I < N; ++I) {
    ItinString += LS;
    ItinString += Name + "Bypass::" + BypassList[I]->getName().str();
  }
  for (; I < NOperandCycles; ++I) {
    ItinString += LS;
    ItinString += " 0";
  }
}

std::string
PrinterCapstone::subtargetGetStageEntryPartI(std::string const &ItinStageString,
                                             unsigned StageCount) const {
  return ItinStageString + ", // " + itostr(StageCount);
}
std::string
PrinterCapstone::subtargetGetStageEntryPartII(unsigned StageCount,
                                              unsigned NStages) const {
  return "-" + itostr(StageCount + NStages - 1);
}
std::string PrinterCapstone::subtargetGetStageEntryPartIII() const {
  return "\n";
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartI(
    std::string const &ItinOperandCycleString) const {
  return ItinOperandCycleString + ", // ";
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartII(
    unsigned OperandCycleCount, unsigned NOperandCycles) const {
  return "-" + itostr(OperandCycleCount + NOperandCycles - 1);
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartIII(
    std::string const &OperandIdxComment) const {
  return OperandIdxComment + "\n";
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartIV(
    std::string const &ItinBypassString,
    std::string const &OperandIdxComment) const {
  return ItinBypassString + ", // " + OperandIdxComment + "\n";
}

void PrinterCapstone::subtargetEmitProcessorItineraryTable(
    std::string const &ItinsDefName, std::vector<InstrItinerary> &ItinList,
    CodeGenSchedModels const &SchedModels) const {
  OS << "\n";
  OS << "static const llvm::InstrItinerary ";

  // Begin processor itinerary table
  OS << ItinsDefName << "[] = {\n";

  // For each itinerary class in CodeGenSchedClass::Index order.
  for (unsigned J = 0, M = ItinList.size(); J < M; ++J) {
    InstrItinerary const &Intinerary = ItinList[J];

    // Emit Itinerary in the form of
    // { firstStage, lastStage, firstCycle, lastCycle } // index
    OS << "  { " << Intinerary.NumMicroOps << ", " << Intinerary.FirstStage
       << ", " << Intinerary.LastStage << ", " << Intinerary.FirstOperandCycle
       << ", " << Intinerary.LastOperandCycle << " }"
       << ", // " << J << " " << SchedModels.getSchedClass(J).Name << "\n";
  }
  // End processor itinerary table
  OS << "  { 0, uint16_t(~0U), uint16_t(~0U), uint16_t(~0U), uint16_t(~0U) }"
        "// end marker\n";
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitPreOperandTableComment() const {
  OS << "\n// ===============================================================\n"
     << "// Data tables for the new per-operand machine model.\n";
}

// Emit SchedClass tables for all processors and associated global tables.
void PrinterCapstone::subtargetEmitSchedClassTables(
    SchedClassTablesT &SchedTables, std::string const &TargetName,
    CodeGenSchedModels const &SchedModels) const {
  // Emit global WriteProcResTable.
  OS << "\n// {ProcResourceIdx, Cycles}\n"
     << "extern const llvm::MCWriteProcResEntry " << TargetName
     << "WriteProcResTable[] = {\n"
     << "  { 0,  0}, // Invalid\n";
  for (unsigned WPRIdx = 1, WPREnd = SchedTables.WriteProcResources.size();
       WPRIdx != WPREnd; ++WPRIdx) {
    MCWriteProcResEntry const &WPREntry =
        SchedTables.WriteProcResources[WPRIdx];
    OS << "  {" << format("%2d", WPREntry.ProcResourceIdx) << ", "
       << format("%2d", WPREntry.Cycles) << "}";
    if (WPRIdx + 1 < WPREnd)
      OS << ',';
    OS << " // #" << WPRIdx << '\n';
  }
  OS << "}; // " << TargetName << "WriteProcResTable\n";

  // Emit global WriteLatencyTable.
  OS << "\n// {Cycles, WriteResourceID}\n"
     << "extern const llvm::MCWriteLatencyEntry " << TargetName
     << "WriteLatencyTable[] = {\n"
     << "  { 0,  0}, // Invalid\n";
  for (unsigned WLIdx = 1, WLEnd = SchedTables.WriteLatencies.size();
       WLIdx != WLEnd; ++WLIdx) {
    MCWriteLatencyEntry &WLEntry = SchedTables.WriteLatencies[WLIdx];
    OS << "  {" << format("%2d", WLEntry.Cycles) << ", "
       << format("%2d", WLEntry.WriteResourceID) << "}";
    if (WLIdx + 1 < WLEnd)
      OS << ',';
    OS << " // #" << WLIdx << " " << SchedTables.WriterNames[WLIdx] << '\n';
  }
  OS << "}; // " << TargetName << "WriteLatencyTable\n";

  // Emit global ReadAdvanceTable.
  OS << "\n// {UseIdx, WriteResourceID, Cycles}\n"
     << "extern const llvm::MCReadAdvanceEntry " << TargetName
     << "ReadAdvanceTable[] = {\n"
     << "  {0,  0,  0}, // Invalid\n";
  for (unsigned RAIdx = 1, RAEnd = SchedTables.ReadAdvanceEntries.size();
       RAIdx != RAEnd; ++RAIdx) {
    MCReadAdvanceEntry &RAEntry = SchedTables.ReadAdvanceEntries[RAIdx];
    OS << "  {" << RAEntry.UseIdx << ", "
       << format("%2d", RAEntry.WriteResourceID) << ", "
       << format("%2d", RAEntry.Cycles) << "}";
    if (RAIdx + 1 < RAEnd)
      OS << ',';
    OS << " // #" << RAIdx << '\n';
  }
  OS << "}; // " << TargetName << "ReadAdvanceTable\n";

  // Emit a SchedClass table for each processor.
  for (CodeGenSchedModels::ProcIter PI = SchedModels.procModelBegin(),
                                    PM = SchedModels.procModelEnd();
       PI != PM; ++PI) {
    if (!PI->hasInstrSchedModel())
      continue;

    std::vector<MCSchedClassDesc> &SCTab =
        SchedTables.ProcSchedClasses[1 + (PI - SchedModels.procModelBegin())];

    OS << "\n// {Name, NumMicroOps, BeginGroup, EndGroup, RetireOOO,"
       << " WriteProcResIdx,#, WriteLatencyIdx,#, ReadAdvanceIdx,#}\n";
    OS << "static const llvm::MCSchedClassDesc " << PI->ModelName
       << "SchedClasses[] = {\n";

    // The first class is always invalid. We no way to distinguish it except by
    // name and position.
    assert(SchedModels.getSchedClass(0).Name == "NoInstrModel" &&
           "invalid class not first");
    OS << "  {DBGFIELD(\"InvalidSchedClass\")  "
       << MCSchedClassDesc::InvalidNumMicroOps
       << ", false, false, false, 0, 0,  0, 0,  0, 0},\n";

    for (unsigned SCIdx = 1, SCEnd = SCTab.size(); SCIdx != SCEnd; ++SCIdx) {
      MCSchedClassDesc &MCDesc = SCTab[SCIdx];
      const CodeGenSchedClass &SchedClass = SchedModels.getSchedClass(SCIdx);
      OS << "  {DBGFIELD(\"" << SchedClass.Name << "\") ";
      if (SchedClass.Name.size() < 18)
        OS.indent(18 - SchedClass.Name.size());
      OS << MCDesc.NumMicroOps << ", " << (MCDesc.BeginGroup ? "true" : "false")
         << ", " << (MCDesc.EndGroup ? "true" : "false") << ", "
         << (MCDesc.RetireOOO ? "true" : "false") << ", "
         << format("%2d", MCDesc.WriteProcResIdx) << ", "
         << MCDesc.NumWriteProcResEntries << ", "
         << format("%2d", MCDesc.WriteLatencyIdx) << ", "
         << MCDesc.NumWriteLatencyEntries << ", "
         << format("%2d", MCDesc.ReadAdvanceIdx) << ", "
         << MCDesc.NumReadAdvanceEntries << "}, // #" << SCIdx << '\n';
    }
    OS << "}; // " << PI->ModelName << "SchedClasses\n";
  }
}

unsigned PrinterCapstone::subtargetEmitRegisterFileTables(
    CodeGenProcModel const &ProcModel) const {
  // Print the RegisterCost table first.
  OS << "\n// {RegisterClassID, Register Cost, AllowMoveElimination }\n";
  OS << "static const llvm::MCRegisterCostEntry " << ProcModel.ModelName
     << "RegisterCosts"
     << "[] = {\n";

  for (const CodeGenRegisterFile &RF : ProcModel.RegisterFiles) {
    // Skip register files with a default cost table.
    if (RF.hasDefaultCosts())
      continue;
    // Add entries to the cost table.
    for (const CodeGenRegisterCost &RC : RF.Costs) {
      OS << "  { ";
      Record *Rec = RC.RCDef;
      if (Rec->getValue("Namespace"))
        OS << Rec->getValueAsString("Namespace") << "::";
      OS << Rec->getName() << "RegClassID, " << RC.Cost << ", "
         << RC.AllowMoveElimination << "},\n";
    }
  }
  OS << "};\n";

  // Now generate a table with register file info.
  OS << "\n // {Name, #PhysRegs, #CostEntries, IndexToCostTbl, "
     << "MaxMovesEliminatedPerCycle, AllowZeroMoveEliminationOnly }\n";
  OS << "static const llvm::MCRegisterFileDesc " << ProcModel.ModelName
     << "RegisterFiles"
     << "[] = {\n"
     << "  { \"InvalidRegisterFile\", 0, 0, 0, 0, 0 },\n";
  unsigned CostTblIndex = 0;

  for (const CodeGenRegisterFile &RD : ProcModel.RegisterFiles) {
    OS << "  { ";
    OS << '"' << RD.Name << '"' << ", " << RD.NumPhysRegs << ", ";
    unsigned NumCostEntries = RD.Costs.size();
    OS << NumCostEntries << ", " << CostTblIndex << ", "
       << RD.MaxMovesEliminatedPerCycle << ", "
       << RD.AllowZeroMoveEliminationOnly << "},\n";
    CostTblIndex += NumCostEntries;
  }
  OS << "};\n";

  return CostTblIndex;
}

void PrinterCapstone::subtargetEmitMCExtraProcInfoTableHeader(
    std::string const &ProcModelName) const {
  OS << "\nstatic const llvm::MCExtraProcessorInfo " << ProcModelName
     << "ExtraInfo = {\n  ";
}

void PrinterCapstone::subtargetEmitMCExtraProcInfoTableEnd() const {
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitReorderBufferSize(
    int64_t ReorderBufferSize) const {
  OS << ReorderBufferSize << ", // ReorderBufferSize\n  ";
}

void PrinterCapstone::subtargetEmitMaxRetirePerCycle(
    int64_t MaxRetirePerCycle) const {
  OS << MaxRetirePerCycle << ", // MaxRetirePerCycle\n  ";
}

void PrinterCapstone::subtargetEmitRegisterFileInfo(
    CodeGenProcModel const &ProcModel, unsigned NumRegisterFiles,
    unsigned NumCostEntries) const {
  if (NumRegisterFiles)
    OS << ProcModel.ModelName << "RegisterFiles,\n  " << (1 + NumRegisterFiles);
  else
    OS << "nullptr,\n  0";

  OS << ", // Number of register files.\n  ";
  if (NumCostEntries)
    OS << ProcModel.ModelName << "RegisterCosts,\n  ";
  else
    OS << "nullptr,\n  ";
  OS << NumCostEntries << ", // Number of register cost entries.\n";
}

void PrinterCapstone::subtargetEmitResourceDescriptorLoadQueue(
    unsigned QueueID) const {
  OS << "  " << QueueID << ", // Resource Descriptor for the Load Queue\n";
}

void PrinterCapstone::subtargetEmitResourceDescriptorStoreQueue(
    unsigned QueueID) const {
  OS << "  " << QueueID << ", // Resource Descriptor for the Store Queue\n";
}

void PrinterCapstone::subtargetEmitProcessorResourceSubUnits(
    const CodeGenProcModel &ProcModel,
    CodeGenSchedModels const &SchedModels) const {
  OS << "\nstatic const unsigned " << ProcModel.ModelName
     << "ProcResourceSubUnits[] = {\n"
     << "  0,  // Invalid\n";

  for (unsigned I = 0, E = ProcModel.ProcResourceDefs.size(); I < E; ++I) {
    Record *PRDef = ProcModel.ProcResourceDefs[I];
    if (!PRDef->isSubClassOf("ProcResGroup"))
      continue;
    RecVec const ResUnits = PRDef->getValueAsListOfDefs("Resources");
    for (Record *RUDef : ResUnits) {
      Record *const RU =
          SchedModels.findProcResUnits(RUDef, ProcModel, PRDef->getLoc());
      for (unsigned J = 0; J < RU->getValueAsInt("NumUnits"); ++J) {
        OS << "  " << ProcModel.getProcResourceIdx(RU) << ", ";
      }
    }
    OS << "  // " << PRDef->getName() << "\n";
  }
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitMCProcResourceDescHeader(
    std::string const &ProcModelName) const {
  OS << "\n// {Name, NumUnits, SuperIdx, BufferSize, SubUnitsIdxBegin}\n";
  OS << "static const llvm::MCProcResourceDesc " << ProcModelName
     << "ProcResources"
     << "[] = {\n"
     << "  {\"InvalidUnit\", 0, 0, 0, 0},\n";
}

void PrinterCapstone::subtargetEmitMCProcResourceDescEnd() const {
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitMCProcResourceDesc(
    Record const *PRDef, Record const *SuperDef,
    std::string const &ProcModelName, unsigned SubUnitsOffset,
    unsigned SuperIdx, unsigned NumUnits, int BufferSize, unsigned I,
    unsigned const SubUnitsBeginOffset) const {
  // Emit the ProcResourceDesc
  OS << "  {\"" << PRDef->getName() << "\", ";
  if (PRDef->getName().size() < 15)
    OS.indent(15 - PRDef->getName().size());
  OS << NumUnits << ", " << SuperIdx << ", " << BufferSize << ", ";
  if (SubUnitsBeginOffset != SubUnitsOffset) {
    OS << ProcModelName << "ProcResourceSubUnits + " << SubUnitsBeginOffset;
  } else {
    OS << "nullptr";
  }
  OS << "}, // #" << I + 1;
  if (SuperDef)
    OS << ", Super=" << SuperDef->getName();
  OS << "\n";
}

// Emit either the value defined in the TableGen Record, or the default
// value defined in the C++ header. The Record is null if the processor does not
// define a model.
void PrinterCapstone::subtargetEmitProcessorProp(Record const *R,
                                                 StringRef const Name,
                                                 char Separator) const {
  OS << "  ";
  int const V = R ? R->getValueAsInt(Name) : -1;
  if (V >= 0)
    OS << V << Separator << " // " << Name;
  else
    OS << "MCSchedModel::Default" << Name << Separator;
  OS << '\n';
}

void PrinterCapstone::subtargetEmitProcModelHeader(
    std::string const &ModelName) const {
  OS << "\n";
  OS << "static const llvm::MCSchedModel " << ModelName << " = {\n";
}

void PrinterCapstone::subtargetEmitProcModel(
    CodeGenProcModel const &PM, CodeGenSchedModels const &SchedModels) const {
  bool const PostRAScheduler =
      (PM.ModelDef ? PM.ModelDef->getValueAsBit("PostRAScheduler") : false);

  OS << "  " << (PostRAScheduler ? "true" : "false") << ", // "
     << "PostRAScheduler\n";

  bool const CompleteModel =
      (PM.ModelDef ? PM.ModelDef->getValueAsBit("CompleteModel") : false);

  OS << "  " << (CompleteModel ? "true" : "false") << ", // "
     << "CompleteModel\n";

  OS << "  " << PM.Index << ", // Processor ID\n";
  if (PM.hasInstrSchedModel())
    OS << "  " << PM.ModelName << "ProcResources"
       << ",\n"
       << "  " << PM.ModelName << "SchedClasses"
       << ",\n"
       << "  " << PM.ProcResourceDefs.size() + 1 << ",\n"
       << "  " << (SchedModels.schedClassEnd() - SchedModels.schedClassBegin())
       << ",\n";
  else
    OS << "  nullptr, nullptr, 0, 0,"
       << " // No instruction-level machine model.\n";
  if (PM.hasItineraries())
    OS << "  " << PM.ItinsDef->getName() << ",\n";
  else
    OS << "  nullptr, // No Itinerary\n";
  if (PM.hasExtraProcessorInfo())
    OS << "  &" << PM.ModelName << "ExtraInfo,\n";
  else
    OS << "  nullptr // No extra processor descriptor\n";
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitResolveVariantSchedClassImplHdr() const {
  OS << "unsigned resolveVariantSchedClassImpl(unsigned SchedClass,\n"
     << "    const MCInst *MI, const MCInstrInfo *MCII, unsigned CPUID) {\n";
}

void PrinterCapstone::subtargetEmitResolveVariantSchedClassImplEnd() const {
  OS << "}\n";
}

void PrinterCapstone::subtargetEmitSchedClassSwitch() const {
  OS << "  switch (SchedClass) {\n";
}

void PrinterCapstone::subtargetEmitSchedClassCase(
    unsigned VC, std::string const &SCName) const {
  OS << "  case " << VC << ": // " << SCName << '\n';
}

void PrinterCapstone::subtargetEmitSchedClassProcGuard(
    unsigned Pi, bool OnlyExpandMCInstPredicates,
    std::string const &ModelName) const {
  OS << "    ";

  // Emit a guard on the processor ID.
  if (Pi != 0) {
    OS << (OnlyExpandMCInstPredicates ? "if (CPUID == "
                                      : "if (SchedModel->getProcessorID() == ");
    OS << Pi << ") ";
    OS << "{ // " << ModelName << '\n';
  }
}

// Indent <= -1 (default = -1) means previous PE indent level.
void PrinterCapstone::subtargetEmitPredicates(
    CodeGenSchedTransition const &T, CodeGenSchedClass const &SC,
    bool (*IsTruePredicate)(Record const *Rec), int Indent) const {
  if (Indent > -1)
    PE->setIndentLevel(Indent);
  std::string Buffer;
  raw_string_ostream SS(Buffer);

  // If not all predicates are MCTrue, then we need an if-stmt.
  unsigned const NumNonTruePreds =
      T.PredTerm.size() - count_if(T.PredTerm, IsTruePredicate);

  SS.indent(PE->getIndentLevel() * 2);

  if (NumNonTruePreds) {
    bool FirstNonTruePredicate = true;
    SS << "if (";

    PE->setIndentLevel(PE->getIndentLevel() + 2);

    for (const Record *Rec : T.PredTerm) {
      // Skip predicates that evaluate to "true".
      if (IsTruePredicate(Rec))
        continue;

      if (FirstNonTruePredicate) {
        FirstNonTruePredicate = false;
      } else {
        SS << "\n";
        SS.indent(PE->getIndentLevel() * 2);
        SS << "&& ";
      }

      if (Rec->isSubClassOf("MCSchedPredicate")) {
        PE->expandPredicate(SS, Rec->getValueAsDef("Pred"));
        continue;
      }

      // Expand this legacy predicate and wrap it around braces if there is more
      // than one predicate to expand.
      SS << ((NumNonTruePreds > 1) ? "(" : "")
         << Rec->getValueAsString("Predicate")
         << ((NumNonTruePreds > 1) ? ")" : "");
    }

    SS << ")\n"; // end of if-stmt
    PE->decreaseIndentLevel();
    SS.indent(PE->getIndentLevel() * 2);
    PE->decreaseIndentLevel();
  }

  SS << "return " << T.ToClassIdx << "; // " << SC.Name << '\n';
  OS << Buffer;
}

void PrinterCapstone::subtargetEmitProcTransitionEnd() const {
  OS << "    }\n";
}

void PrinterCapstone::subtargetEmitSchedClassCaseEnd(
    CodeGenSchedClass const &SC) const {
  if (SC.isInferred())
    OS << "    return " << SC.Index << ";\n";
  OS << "    break;\n";
}

void PrinterCapstone::subtargetEmitSchedClassSwitchEnd() const {
  OS << "  };\n";
}

// Used by method `SubtargetEmitter::emitSchedModelHelpersImpl()` to generate
// epilogue code for the auto-generated helper.
void PrinterCapstone::subtargetEmitSchedModelHelperEpilogue(
    bool ShouldReturnZero) const {
  if (ShouldReturnZero) {
    OS << "  // Don't know how to resolve this scheduling class.\n"
       << "  return 0;\n";
    return;
  }

  OS << "  report_fatal_error(\"Expected a variant SchedClass\");\n";
}

void PrinterCapstone::subtargetEmitGenMCSubtargetInfoClass(
    std::string const &TargetName, bool OverrideGetHwMode) const {
  OS << "struct " << TargetName
     << "GenMCSubtargetInfo : public MCSubtargetInfo {\n";
  OS << "  " << TargetName << "GenMCSubtargetInfo(const Triple &TT,\n"
     << "    StringRef CPU, StringRef TuneCPU, StringRef FS,\n"
     << "    ArrayRef<SubtargetFeatureKV> PF,\n"
     << "    ArrayRef<SubtargetSubTypeKV> PD,\n"
     << "    const MCWriteProcResEntry *WPR,\n"
     << "    const MCWriteLatencyEntry *WL,\n"
     << "    const MCReadAdvanceEntry *RA, const InstrStage *IS,\n"
     << "    const unsigned *OC, const unsigned *FP) :\n"
     << "      MCSubtargetInfo(TT, CPU, TuneCPU, FS, PF, PD,\n"
     << "                      WPR, WL, RA, IS, OC, FP) { }\n\n"
     << "  unsigned resolveVariantSchedClass(unsigned SchedClass,\n"
     << "      const MCInst *MI, const MCInstrInfo *MCII,\n"
     << "      unsigned CPUID) const override {\n"
     << "    return " << TargetName << "_MC"
     << "::resolveVariantSchedClassImpl(SchedClass, MI, MCII, CPUID);\n";
  OS << "  }\n";
  if (OverrideGetHwMode)
    OS << "  unsigned getHwMode() const override;\n";
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitMCSubtargetInfoImpl(
    std::string const &TargetName, unsigned NumFeatures, unsigned NumProcs,
    bool SchedModelHasItin) const {
  OS << "\nstatic inline MCSubtargetInfo *create" << TargetName
     << "MCSubtargetInfoImpl("
     << "const Triple &TT, StringRef CPU, StringRef TuneCPU, StringRef FS) {\n";
  OS << "  return new " << TargetName
     << "GenMCSubtargetInfo(TT, CPU, TuneCPU, FS, ";
  if (NumFeatures)
    OS << TargetName << "FeatureKV, ";
  else
    OS << "std::nullopt, ";
  if (NumProcs)
    OS << TargetName << "SubTypeKV, ";
  else
    OS << "None, ";
  OS << '\n';
  OS.indent(22);
  OS << TargetName << "WriteProcResTable, " << TargetName
     << "WriteLatencyTable, " << TargetName << "ReadAdvanceTable, ";
  OS << '\n';
  OS.indent(22);
  if (SchedModelHasItin) {
    OS << TargetName << "Stages, " << TargetName << "OperandCycles, "
       << TargetName << "ForwardingPaths";
  } else
    OS << "nullptr, nullptr, nullptr";
  OS << ");\n}\n\n";
}

void PrinterCapstone::subtargetEmitIncludeSTIDesc() const {
  OS << "#include \"llvm/Support/Debug.h\"\n";
  OS << "#include \"llvm/Support/raw_ostream.h\"\n\n";
}

void PrinterCapstone::subtargetEmitDFAPacketizerClass(
    std::string const &TargetName, std::string const &ClassName,
    bool OverrideGetHwMode) const {
  OS << "class DFAPacketizer;\n";
  OS << "namespace " << TargetName << "_MC {\n"
     << "unsigned resolveVariantSchedClassImpl(unsigned SchedClass,"
     << " const MCInst *MI, const MCInstrInfo *MCII, unsigned CPUID);\n"
     << "} // end namespace " << TargetName << "_MC\n\n";
  OS << "struct " << ClassName << " : public TargetSubtargetInfo {\n"
     << "  explicit " << ClassName << "(const Triple &TT, StringRef CPU, "
     << "StringRef TuneCPU, StringRef FS);\n"
     << "public:\n"
     << "  unsigned resolveSchedClass(unsigned SchedClass, "
     << " const MachineInstr *DefMI,"
     << " const TargetSchedModel *SchedModel) const override;\n"
     << "  unsigned resolveVariantSchedClass(unsigned SchedClass,"
     << " const MCInst *MI, const MCInstrInfo *MCII,"
     << " unsigned CPUID) const override;\n"
     << "  DFAPacketizer *createDFAPacketizer(const InstrItineraryData *IID)"
     << " const;\n";
  if (OverrideGetHwMode)
    OS << "  unsigned getHwMode() const override;\n";
}

void PrinterCapstone::subtargetEmitDFASubtargetInfoImpl(
    std::string const &TargetName, std::string const &ClassName,
    unsigned NumFeatures, unsigned NumProcs, bool SchedModelHasItin) const {
  OS << ClassName << "::" << ClassName << "(const Triple &TT, StringRef CPU, "
     << "StringRef TuneCPU, StringRef FS)\n"
     << "  : TargetSubtargetInfo(TT, CPU, TuneCPU, FS, ";
  if (NumFeatures)
    OS << "makeArrayRef(" << TargetName << "FeatureKV, " << NumFeatures
       << "), ";
  else
    OS << "std::nullopt, ";
  if (NumProcs)
    OS << "makeArrayRef(" << TargetName << "SubTypeKV, " << NumProcs << "), ";
  else
    OS << "None, ";
  OS << '\n';
  OS.indent(24);
  OS << TargetName << "WriteProcResTable, " << TargetName
     << "WriteLatencyTable, " << TargetName << "ReadAdvanceTable, ";
  OS << '\n';
  OS.indent(24);
  if (SchedModelHasItin) {
    OS << TargetName << "Stages, " << TargetName << "OperandCycles, "
       << TargetName << "ForwardingPaths";
  } else
    OS << "nullptr, nullptr, nullptr";
  OS << ") {}\n\n";
}

void PrinterCapstone::subtargetEmitDFAPacketizerClassEnd() const {
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitSTICtor() const {
  OS << "#include \"llvm/CodeGen/TargetSchedule.h\"\n\n";
}

void PrinterCapstone::subtargetEmitExternKVArrays(
    std::string const &TargetName, bool SchedModelsHasItin) const {
  OS << "extern const llvm::SubtargetFeatureKV " << TargetName
     << "FeatureKV[];\n";
  OS << "extern const llvm::SubtargetSubTypeKV " << TargetName
     << "SubTypeKV[];\n";
  OS << "extern const llvm::MCWriteProcResEntry " << TargetName
     << "WriteProcResTable[];\n";
  OS << "extern const llvm::MCWriteLatencyEntry " << TargetName
     << "WriteLatencyTable[];\n";
  OS << "extern const llvm::MCReadAdvanceEntry " << TargetName
     << "ReadAdvanceTable[];\n";

  if (SchedModelsHasItin) {
    OS << "extern const llvm::InstrStage " << TargetName << "Stages[];\n";
    OS << "extern const unsigned " << TargetName << "OperandCycles[];\n";
    OS << "extern const unsigned " << TargetName << "ForwardingPaths[];\n";
  }
}

void PrinterCapstone::subtargetEmitClassDefs(std::string const &TargetName,
                                             std::string const &ClassName,
                                             unsigned NumFeatures,
                                             unsigned NumProcs,
                                             bool SchedModelsHasItin) const {
  OS << ClassName << "::" << ClassName << "(const Triple &TT, StringRef CPU, "
     << "StringRef TuneCPU, StringRef FS)\n"
     << "  : TargetSubtargetInfo(TT, CPU, TuneCPU, FS, ";
  if (NumFeatures)
    OS << "makeArrayRef(" << TargetName << "FeatureKV, " << NumFeatures
       << "), ";
  else
    OS << "None, ";
  if (NumProcs)
    OS << "makeArrayRef(" << TargetName << "SubTypeKV, " << NumProcs << "), ";
  else
    OS << "None, ";
  OS << '\n';
  OS.indent(24);
  OS << TargetName << "WriteProcResTable, " << TargetName
     << "WriteLatencyTable, " << TargetName << "ReadAdvanceTable, ";
  OS << '\n';
  OS.indent(24);
  if (SchedModelsHasItin) {
    OS << TargetName << "Stages, " << TargetName << "OperandCycles, "
       << TargetName << "ForwardingPaths";
  } else
    OS << "nullptr, nullptr, nullptr";
  OS << ") {}\n\n";
}

void PrinterCapstone::subtargetEmitResolveSchedClassHdr(
    std::string const &ClassName) const {
  OS << "unsigned " << ClassName
     << "\n::resolveSchedClass(unsigned SchedClass, const MachineInstr *MI,"
     << " const TargetSchedModel *SchedModel) const {\n";
}

void PrinterCapstone::subtargetEmitResolveSchedClassEnd(
    std::string const &ClassName) const {
  OS << "} // " << ClassName << "::resolveSchedClass\n\n";
}

void PrinterCapstone::subtargetEmitResolveVariantSchedClass(
    std::string const &TargetName, std::string const &ClassName) const {
  OS << "unsigned " << ClassName
     << "\n::resolveVariantSchedClass(unsigned SchedClass, const MCInst *MI,"
     << " const MCInstrInfo *MCII, unsigned CPUID) const {\n"
     << "  return " << TargetName << "_MC"
     << "::resolveVariantSchedClassImpl(SchedClass, MI, MCII, CPUID);\n"
     << "} // " << ClassName << "::resolveVariantSchedClass\n\n";
}

void PrinterCapstone::subtargetEmitPredicateProlog(
    const RecordKeeper &Records) const {
  std::string Buffer;
  raw_string_ostream Stream(Buffer);

  // Collect all the PredicateProlog records and print them to the output
  // stream.
  std::vector<Record *> Prologs =
      Records.getAllDerivedDefinitions("PredicateProlog");
  llvm::sort(Prologs, LessRecord());
  for (Record *P : Prologs)
    Stream << P->getValueAsString("Code") << '\n';

  OS << Buffer;
}

void PrinterCapstone::subtargetEmitParseFeaturesFunction(
    std::string const &TargetName,
    std::vector<Record *> const &Features) const {
  OS << "// ParseSubtargetFeatures - Parses features string setting specified\n"
     << "// subtarget options.\n"
     << "void llvm::";
  OS << TargetName;
  OS << "Subtarget::ParseSubtargetFeatures(StringRef CPU, StringRef TuneCPU, "
     << "StringRef FS) {\n"
     << "  LLVM_DEBUG(dbgs() << \"\\nFeatures:\" << FS);\n"
     << "  LLVM_DEBUG(dbgs() << \"\\nCPU:\" << CPU);\n"
     << "  LLVM_DEBUG(dbgs() << \"\\nTuneCPU:\" << TuneCPU << \"\\n\\n\");\n";

  if (Features.empty()) {
    OS << "}\n";
    return;
  }

  OS << "  InitMCProcessorInfo(CPU, TuneCPU, FS);\n"
     << "  const FeatureBitset &Bits = getFeatureBits();\n";

  for (Record *R : Features) {
    // Next record
    StringRef const Instance = R->getName();
    StringRef const Value = R->getValueAsString("Value");
    StringRef const Attribute = R->getValueAsString("Attribute");

    if (Value == "true" || Value == "false")
      OS << "  if (Bits[" << TargetName << "::" << Instance << "]) "
         << Attribute << " = " << Value << ";\n";
    else
      OS << "  if (Bits[" << TargetName << "::" << Instance << "] && "
         << Attribute << " < " << Value << ") " << Attribute << " = " << Value
         << ";\n";
  }

  OS << "}\n";
}

void PrinterCapstone::subtargetEmitExpandedSTIPreds(
    StringRef const &TargetName, std::string const &ClassName,
    CodeGenSchedModels const &SchedModels) {
  initNewPE(TargetName);
  PE->setClassPrefix(ClassName);
  PE->setExpandDefinition(true);
  PE->setByRef(false);
  PE->setIndentLevel(0);

  for (const STIPredicateFunction &Fn : SchedModels.getSTIPredicates())
    PE->expandSTIPredicate(OS, Fn);
}

void PrinterCapstone::subtargetPrepareSchedClassPreds(
    StringRef const &TargetName, bool OnlyExpandMCInstPredicates) {
  initNewPE(TargetName);
  PE->setByRef(false);
  PE->setExpandForMC(OnlyExpandMCInstPredicates);
}

void PrinterCapstone::subtargetEmitExpandedSTIPredsMCAnaDecl(
    StringRef const &TargetName, CodeGenSchedModels const &SchedModels) {
  initNewPE(TargetName);
  PE->setExpandForMC(true);
  PE->setByRef(true);
  for (const STIPredicateFunction &Fn : SchedModels.getSTIPredicates())
    PE->expandSTIPredicate(OS, Fn);
}

void PrinterCapstone::subtargetEmitExpandedSTIPredsMCAnaDefs(
    StringRef const &TargetName, std::string const &ClassPrefix,
    CodeGenSchedModels const &SchedModels) const {
  // Predicate expander was initialized before.
  PE->setExpandDefinition(true);
  PE->setClassPrefix(ClassPrefix);
  PE->setIndentLevel(0);
  for (const STIPredicateFunction &Fn : SchedModels.getSTIPredicates())
    PE->expandSTIPredicate(OS, Fn);
}

void PrinterCapstone::subtargetEmitExpandedSTIPredsHeader(
    StringRef const &TargetName, CodeGenSchedModels const &SchedModels) {
  initNewPE(TargetName);
  PE->setByRef(false);
  for (const STIPredicateFunction &Fn : SchedModels.getSTIPredicates())
    PE->expandSTIPredicate(OS, Fn);
}

void PrinterCapstone::subtargetEmitStageAndSycleTables(
    std::string const &StageTable, std::string const &OperandCycleTable,
    std::string const &BypassTable) const {
  OS << StageTable;
  OS << OperandCycleTable;
  OS << BypassTable;
}

//---------------------------
// Backend: InstrInfoEmitter
//---------------------------

void PrinterCapstone::instrInfoEmitSourceFileHeader() const {
  emitSourceFileHeader("Target Instruction Enum Values and Descriptors", OS);
}

void PrinterCapstone::instrInfoPrintDefList(
    const std::vector<Record *> &Uses, unsigned Num,
    std::string (*GetQualifiedName)(Record const *R)) const {
  OS << "static const MCPhysReg ImplicitList" << Num << "[] = { ";
  for (Record *U : Uses)
    OS << GetQualifiedName(U) << ", ";
  OS << "0 };\n";
}

void PrinterCapstone::instrInfoEmitOperandInfoTabe(
    std::vector<std::string> const &OperandInfo, unsigned N) const {
  OS << "static const MCOperandInfo OperandInfo" << N << "[] = { ";
  for (const std::string &Info : OperandInfo)
    OS << "{ " << Info << " }, ";
  OS << "};\n";
}

void PrinterCapstone::instrInfoEmitMCInstrDescHdr(
    std::string TargetName) const {
  OS << "\nextern const MCInstrDesc " << TargetName << "Insts[] = {\n";
}

void PrinterCapstone::instrInfoEmitMCInstrDescEnd() const { OS << "};\n\n"; }

void PrinterCapstone::instrInfoEmitRecord(CodeGenSchedModels const &SchedModels,
                                          CodeGenInstruction const &Inst,
                                          unsigned Num, int MinOperands) const {
  OS << "  { ";
  OS << Num << ",\t" << MinOperands << ",\t" << Inst.Operands.NumDefs << ",\t"
     << Inst.TheDef->getValueAsInt("Size") << ",\t"
     << SchedModels.getSchedClassIdx(Inst) << ",\t0";
}

void PrinterCapstone::instrInfoEmitTargetIndepFlags(
    CodeGenInstruction const &Inst, bool GetAllowRegisterRenaming) const {
  // clang-format off
  if (Inst.isPreISelOpcode)    OS << "|(1ULL<<MCID::PreISelOpcode)";
  if (Inst.isPseudo)           OS << "|(1ULL<<MCID::Pseudo)";
  if (Inst.isMeta)             OS << "|(1ULL<<MCID::Meta)";
  if (Inst.isReturn)           OS << "|(1ULL<<MCID::Return)";
  if (Inst.isEHScopeReturn)    OS << "|(1ULL<<MCID::EHScopeReturn)";
  if (Inst.isBranch)           OS << "|(1ULL<<MCID::Branch)";
  if (Inst.isIndirectBranch)   OS << "|(1ULL<<MCID::IndirectBranch)";
  if (Inst.isCompare)          OS << "|(1ULL<<MCID::Compare)";
  if (Inst.isMoveImm)          OS << "|(1ULL<<MCID::MoveImm)";
  if (Inst.isMoveReg)          OS << "|(1ULL<<MCID::MoveReg)";
  if (Inst.isBitcast)          OS << "|(1ULL<<MCID::Bitcast)";
  if (Inst.isAdd)              OS << "|(1ULL<<MCID::Add)";
  if (Inst.isTrap)             OS << "|(1ULL<<MCID::Trap)";
  if (Inst.isSelect)           OS << "|(1ULL<<MCID::Select)";
  if (Inst.isBarrier)          OS << "|(1ULL<<MCID::Barrier)";
  if (Inst.hasDelaySlot)       OS << "|(1ULL<<MCID::DelaySlot)";
  if (Inst.isCall)             OS << "|(1ULL<<MCID::Call)";
  if (Inst.canFoldAsLoad)      OS << "|(1ULL<<MCID::FoldableAsLoad)";
  if (Inst.mayLoad)            OS << "|(1ULL<<MCID::MayLoad)";
  if (Inst.mayStore)           OS << "|(1ULL<<MCID::MayStore)";
  if (Inst.mayRaiseFPException) OS << "|(1ULL<<MCID::MayRaiseFPException)";
  if (Inst.isPredicable)       OS << "|(1ULL<<MCID::Predicable)";
  if (Inst.isConvertibleToThreeAddress) OS << "|(1ULL<<MCID::ConvertibleTo3Addr)";
  if (Inst.isCommutable)       OS << "|(1ULL<<MCID::Commutable)";
  if (Inst.isTerminator)       OS << "|(1ULL<<MCID::Terminator)";
  if (Inst.isReMaterializable) OS << "|(1ULL<<MCID::Rematerializable)";
  if (Inst.isNotDuplicable)    OS << "|(1ULL<<MCID::NotDuplicable)";
  if (Inst.Operands.hasOptionalDef) OS << "|(1ULL<<MCID::HasOptionalDef)";
  if (Inst.usesCustomInserter) OS << "|(1ULL<<MCID::UsesCustomInserter)";
  if (Inst.hasPostISelHook)    OS << "|(1ULL<<MCID::HasPostISelHook)";
  if (Inst.Operands.isVariadic)OS << "|(1ULL<<MCID::Variadic)";
  if (Inst.hasSideEffects)     OS << "|(1ULL<<MCID::UnmodeledSideEffects)";
  if (Inst.isAsCheapAsAMove)   OS << "|(1ULL<<MCID::CheapAsAMove)";
  if (!GetAllowRegisterRenaming || Inst.hasExtraSrcRegAllocReq)
    OS << "|(1ULL<<MCID::ExtraSrcRegAllocReq)";
  if (!GetAllowRegisterRenaming || Inst.hasExtraDefRegAllocReq)
    OS << "|(1ULL<<MCID::ExtraDefRegAllocReq)";
  if (Inst.isRegSequence) OS << "|(1ULL<<MCID::RegSequence)";
  if (Inst.isExtractSubreg) OS << "|(1ULL<<MCID::ExtractSubreg)";
  if (Inst.isInsertSubreg) OS << "|(1ULL<<MCID::InsertSubreg)";
  if (Inst.isConvergent) OS << "|(1ULL<<MCID::Convergent)";
  if (Inst.variadicOpsAreDefs) OS << "|(1ULL<<MCID::VariadicOpsAreDefs)";
  if (Inst.isAuthenticated) OS << "|(1ULL<<MCID::Authenticated)";
  // clang-format on
}

void PrinterCapstone::instrInfoEmitTSFFlags(uint64_t Value) const {
  OS << ", 0x";
  OS.write_hex(Value);
  OS << "ULL, ";
}

void PrinterCapstone::instrInfoEmitUseDefsLists(
    std::map<std::vector<Record *>, unsigned> &EmittedLists,
    std::vector<Record *> const &UseList,
    std::vector<Record *> const &DefList) const {
  if (UseList.empty())
    OS << "nullptr, ";
  else
    OS << "ImplicitList" << EmittedLists[UseList] << ", ";

  if (DefList.empty())
    OS << "nullptr, ";
  else
    OS << "ImplicitList" << EmittedLists[DefList] << ", ";
}

void PrinterCapstone::instrInfoEmitOperandInfo(
    std::vector<std::string> const &OperandInfo,
    OperandInfoMapTy const &OpInfo) const {
  if (OperandInfo.empty())
    OS << "nullptr";
  else
    OS << "OperandInfo" << OpInfo.find(OperandInfo)->second;
}

void PrinterCapstone::instrInfoEmitRecordEnd(
    unsigned InstNum, std::string const &InstName) const {
  OS << " },  // Inst #" << InstNum << " = " << InstName << "\n";
}

void PrinterCapstone::instrInfoEmitStringLiteralDef(
    std::string const &TargetName,
    SequenceToOffsetTable<std::string> InstrNames) const {
  InstrNames.emitStringLiteralDef(OS, Twine("extern const char ") + TargetName +
                                          "InstrNameData[]");
}

void PrinterCapstone::instrInfoEmitInstrNameIndices(
    std::string const &TargetName,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions,
    SequenceToOffsetTable<std::string> const &InstrNames) const {
  OS << "extern const unsigned " << TargetName << "InstrNameIndices[] = {";
  unsigned Num = 0;
  for (const CodeGenInstruction *Inst : NumberedInstructions) {
    // Newline every eight entries.
    if (Num % 8 == 0)
      OS << "\n    ";
    OS << InstrNames.get(std::string(Inst->TheDef->getName())) << "U, ";
    ++Num;
  }
  OS << "\n};\n\n";
}

void PrinterCapstone::instrInfoEmitInstrDeprFeatures(
    std::string const &TargetName, std::string const &TargetNamespace,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions,
    SequenceToOffsetTable<std::string> const &InstrNames) const {
  OS << "extern const uint8_t " << TargetName
     << "InstrDeprecationFeatures[] = {";
  unsigned Num = 0;
  for (const CodeGenInstruction *Inst : NumberedInstructions) {
    if (Num % 8 == 0)
      OS << "\n    ";
    if (!Inst->HasComplexDeprecationPredicate &&
        !Inst->DeprecatedReason.empty())
      OS << TargetNamespace << "::" << Inst->DeprecatedReason << ", ";
    else
      OS << "uint8_t(-1), ";
    ++Num;
  }
  OS << "\n};\n\n";
}

void PrinterCapstone::instrInfoEmitInstrComplexDeprInfos(
    std::string const &TargetName,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions) const {
  OS << "extern const MCInstrInfo::ComplexDeprecationPredicate " << TargetName
     << "InstrComplexDeprecationInfos[] = {";
  unsigned Num = 0;
  for (const CodeGenInstruction *Inst : NumberedInstructions) {
    if (Num % 8 == 0)
      OS << "\n    ";
    if (Inst->HasComplexDeprecationPredicate)
      // Emit a function pointer to the complex predicate method.
      OS << "&get" << Inst->DeprecatedReason << "DeprecationInfo, ";
    else
      OS << "nullptr, ";
    ++Num;
  }
  OS << "\n};\n\n";
}

void PrinterCapstone::instrInfoEmitMCInstrInfoInitRoutine(
    std::string const &TargetName, unsigned NumberedInstrSize,
    bool HasDeprecationFeatures, bool HasComplexDeprecationInfos) const {
  OS << "static inline void Init" << TargetName
     << "MCInstrInfo(MCInstrInfo *II) {\n";
  OS << "  II->InitMCInstrInfo(" << TargetName << "Insts, " << TargetName
     << "InstrNameIndices, " << TargetName << "InstrNameData, ";
  if (HasDeprecationFeatures)
    OS << TargetName << "InstrDeprecationFeatures, ";
  else
    OS << "nullptr, ";
  if (HasComplexDeprecationInfos)
    OS << TargetName << "InstrComplexDeprecationInfos, ";
  else
    OS << "nullptr, ";
  OS << NumberedInstrSize << ");\n}\n\n";
}

void PrinterCapstone::instrInfoEmitClassStruct(
    std::string const &ClassName) const {
  OS << "struct " << ClassName << " : public TargetInstrInfo {\n"
     << "  explicit " << ClassName
     << "(int CFSetupOpcode = -1, int CFDestroyOpcode = -1, int CatchRetOpcode "
        "= -1, int ReturnOpcode = -1);\n"
     << "  ~" << ClassName << "() override = default;\n";
  OS << "\n};\n";
}

void PrinterCapstone::instrInfoEmitTIIHelperMethod(
    StringRef const &TargetName, Record const *Rec,
    bool ExpandDefinition) const {
  OS << (ExpandDefinition ? "" : "static ") << "bool ";
  if (ExpandDefinition)
    OS << TargetName << "InstrInfo::";
  OS << Rec->getValueAsString("FunctionName");
  OS << "(const MachineInstr &MI)";
  if (!ExpandDefinition) {
    OS << ";\n";
    return;
  }

  OS << " {\n";
  OS.indent(PE->getIndentLevel() * 2);
  PE->expandStatement(OS, Rec->getValueAsDef("Body"));
  OS << "\n}\n\n";
}

void PrinterCapstone::instrInfoEmitExternArrays(
    std::string const &TargetName, bool HasDeprecationFeatures,
    bool HasComplexDeprecationInfos) const {
  OS << "extern const MCInstrDesc " << TargetName << "Insts[];\n";
  OS << "extern const unsigned " << TargetName << "InstrNameIndices[];\n";
  OS << "extern const char " << TargetName << "InstrNameData[];\n";
  if (HasDeprecationFeatures)
    OS << "extern const uint8_t " << TargetName
       << "InstrDeprecationFeatures[];\n";
  if (HasComplexDeprecationInfos)
    OS << "extern const MCInstrInfo::ComplexDeprecationPredicate " << TargetName
       << "InstrComplexDeprecationInfos[];\n";
}

void PrinterCapstone::instrInfoEmitMCInstrInfoInit(
    std::string const &TargetName, std::string const &ClassName,
    unsigned NumberedInstrSize, bool HasDeprecationFeatures,
    bool HasComplexDeprecationInfos) const {
  OS << ClassName << "::" << ClassName
     << "(int CFSetupOpcode, int CFDestroyOpcode, int CatchRetOpcode, int "
        "ReturnOpcode)\n"
     << "  : TargetInstrInfo(CFSetupOpcode, CFDestroyOpcode, CatchRetOpcode, "
        "ReturnOpcode) {\n"
     << "  InitMCInstrInfo(" << TargetName << "Insts, " << TargetName
     << "InstrNameIndices, " << TargetName << "InstrNameData, ";
  if (HasDeprecationFeatures)
    OS << TargetName << "InstrDeprecationFeatures, ";
  else
    OS << "nullptr, ";
  if (HasComplexDeprecationInfos)
    OS << TargetName << "InstrComplexDeprecationInfos, ";
  else
    OS << "nullptr, ";
  OS << NumberedInstrSize << ");\n}\n";
}

void PrinterCapstone::instrInfoEmitOperandEnum(
    std::map<std::string, unsigned> const &Operands) const {
  OS << "enum {\n";
  for (const auto &Op : Operands)
    OS << "  " << Op.first << " = " << Op.second << ",\n";

  OS << "  OPERAND_LAST";
  OS << "\n};\n";
}

void PrinterCapstone::instrInfoEmitGetNamedOperandIdx(
    std::map<std::string, unsigned> const &Operands,
    OpNameMapTy const &OperandMap) const {
  OS << "LLVM_READONLY\n";
  OS << "int16_t getNamedOperandIdx(uint16_t Opcode, uint16_t NamedIdx) {\n";
  if (!Operands.empty()) {
    OS << "  static const int16_t OperandMap [][" << Operands.size()
       << "] = {\n";
    for (const auto &Entry : OperandMap) {
      const std::map<unsigned, unsigned> &OpList = Entry.first;
      OS << "{";

      // Emit a row of the OperandMap table
      for (unsigned I = 0, E = Operands.size(); I != E; ++I)
        OS << (OpList.count(I) == 0 ? -1 : (int)OpList.find(I)->second) << ", ";

      OS << "},\n";
    }
    OS << "};\n";

    OS << "  switch(Opcode) {\n";
    unsigned TableIndex = 0;
    for (const auto &Entry : OperandMap) {
      for (const std::string &Name : Entry.second)
        OS << "  case " << Name << ":\n";

      OS << "    return OperandMap[" << TableIndex++ << "][NamedIdx];\n";
    }
    OS << "  default: return -1;\n";
    OS << "  }\n";
  } else {
    // There are no operands, so no need to emit anything
    OS << "  return -1;\n";
  }
  OS << "}\n";
}

void PrinterCapstone::instrInfoEmitOpTypeEnumPartI() const {
  OS << "enum OperandType {\n";
}

void PrinterCapstone::instrInfoEmitOpTypeEnumPartII(StringRef const &OpName,
                                                    unsigned EnumVal) const {
  OS << "  " << OpName << " = " << EnumVal << ",\n";
}

void PrinterCapstone::instrInfoEmitOpTypeEnumPartIII() const {
  OS << "  OPERAND_TYPE_LIST_END"
     << "\n};\n";
}

void PrinterCapstone::instrInfoEmitOpTypeOffsetTable(
    std::vector<int> OperandOffsets, unsigned OpRecSize,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions) const {
  OS << ((OpRecSize <= UINT16_MAX) ? "  const uint16_t" : "  const uint32_t");
  OS << " Offsets[] = {\n";
  for (int I = 0, E = OperandOffsets.size(); I != E; ++I) {
    OS << "    /* " << NumberedInstructions[I]->TheDef->getName() << " */\n";
    OS << "    " << OperandOffsets[I] << ",\n";
  }
  OS << "  };\n";
}

void PrinterCapstone::instrInfoEmitOpcodeOpTypesTable(
    unsigned EnumVal, std::vector<Record *> const &OperandRecords,
    std::vector<int> OperandOffsets,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions) const {
  OS << "\n  using namespace OpTypes;\n";
  OS << ((EnumVal <= INT8_MAX) ? "  const int8_t" : "  const int16_t");
  OS << " OpcodeOperandTypes[] = {\n    ";
  for (int I = 0, E = OperandRecords.size(), CurOffset = 0; I != E; ++I) {
    // We print each Opcode's operands in its own row.
    if (I == OperandOffsets[CurOffset]) {
      OS << "\n    /* " << NumberedInstructions[CurOffset]->TheDef->getName()
         << " */\n    ";
      while (OperandOffsets[++CurOffset] == I)
        OS << "/* " << NumberedInstructions[CurOffset]->TheDef->getName()
           << " */\n    ";
    }
    Record *OpR = OperandRecords[I];
    if ((OpR->isSubClassOf("Operand") || OpR->isSubClassOf("RegisterOperand") ||
         OpR->isSubClassOf("RegisterClass")) &&
        !OpR->isAnonymous())
      OS << OpR->getName();
    else
      OS << -1;
    OS << ", ";
  }
  OS << "\n  };\n";
}

void PrinterCapstone::instrInfoEmitGetOpTypeHdr() const {
  OS << "LLVM_READONLY\n";
  OS << "static int getOperandType(uint16_t Opcode, uint16_t OpIdx) {\n";
}

void PrinterCapstone::instrInfoEmitGetOpTypeReturn() const {
  OS << "  return OpcodeOperandTypes[Offsets[Opcode] + OpIdx];\n";
}

void PrinterCapstone::instrInfoEmitGetOpTypeUnreachable() const {
  OS << "  llvm_unreachable(\"No instructions defined\");\n";
}

void PrinterCapstone::instrInfoEmitGetOpTypeEnd() const { OS << "}\n"; }

void PrinterCapstone::instrInfoEmitGetMemOpSizeHdr() const {
  OS << "LLVM_READONLY\n";
  OS << "static int getMemOperandSize(int OpType) {\n";
  OS << "  switch (OpType) {\n";
}

void PrinterCapstone::instrInfoEmitGetOpMemSizeTbl(
    std::map<int, std::vector<StringRef>> const &SizeToOperandName) const {
  OS << "  default: return 0;\n";
  for (auto KV : SizeToOperandName) {
    for (const StringRef &OperandName : KV.second)
      OS << "  case OpTypes::" << OperandName << ":\n";
    OS << "    return " << KV.first << ";\n\n";
  }
  OS << "  }\n}\n";
}

std::string
PrinterCapstone::instrInfoGetInstMapEntry(StringRef const &Namespace,
                                          StringRef const &InstrName) const {
  return Namespace.str() + "::" + InstrName.str();
}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeHdr() const {
  OS << "LLVM_READONLY static unsigned\n";
  OS << "getLogicalOperandSize(uint16_t Opcode, uint16_t LogicalOpIdx) {\n";
}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeTable(
    size_t LogicalOpListSize,
    std::vector<const std::vector<unsigned> *> const &LogicalOpSizeList) const {
  OS << "  static const unsigned SizeMap[][" << LogicalOpListSize << "] = {\n";
  for (auto &R : LogicalOpSizeList) {
    const auto &Row = *R;
    OS << "   {";
    int I;
    for (I = 0; I < static_cast<int>(Row.size()); ++I) {
      OS << Row[I] << ", ";
    }
    for (; I < static_cast<int>(LogicalOpListSize); ++I) {
      OS << "0, ";
    }
    OS << "}, ";
    OS << "\n";
  }
  OS << "  };\n";
}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeSwitch(
    std::map<unsigned, std::vector<std::string>> InstMap) const {
  OS << "  switch (Opcode) {\n";
  OS << "  default: return LogicalOpIdx;\n";
  for (auto &P : InstMap) {
    auto OpMapIdx = P.first;
    const auto &Insts = P.second;
    for (const auto &Inst : Insts) {
      OS << "  case " << Inst << ":\n";
    }
    OS << "    return SizeMap[" << OpMapIdx << "][LogicalOpIdx];\n";
  }
  OS << "  }\n";
}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeReturn() const {
  OS << "  return LogicalOpIdx;\n";
}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeEnd() const { OS << "}\n"; }

void PrinterCapstone::instrInfoEmitGetLogicalOpIdx() const {
  OS << "LLVM_READONLY static inline unsigned\n";
  OS << "getLogicalOperandIdx(uint16_t Opcode, uint16_t LogicalOpIdx) {\n";
  OS << "  auto S = 0U;\n";
  OS << "  for (auto i = 0U; i < LogicalOpIdx; ++i)\n";
  OS << "    S += getLogicalOperandSize(Opcode, i);\n";
  OS << "  return S;\n";
  OS << "}\n";
}

std::string
PrinterCapstone::instrInfoGetOpTypeListEntry(StringRef const &Namespace,
                                             StringRef const &OpName) const {
  return Namespace.str() + "::OpTypes::" + OpName.str();
}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeHdr() const {
  OS << "LLVM_READONLY static int\n";
  OS << "getLogicalOperandType(uint16_t Opcode, uint16_t LogicalOpIdx) {\n";
}
void PrinterCapstone::instrInfoEmitGetLogicalOpTypeTable(
    size_t OpTypeListSize,
    std::vector<const std::vector<std::string> *> const &LogicalOpTypeList)
    const {
  OS << "  static const int TypeMap[][" << OpTypeListSize << "] = {\n";
  for (int R = 0, Rs = LogicalOpTypeList.size(); R < Rs; ++R) {
    const auto &Row = *LogicalOpTypeList[R];
    OS << "   {";
    int I, S = Row.size();
    for (I = 0; I < S; ++I) {
      if (I > 0)
        OS << ", ";
      OS << Row[I];
    }
    for (; I < static_cast<int>(OpTypeListSize); ++I) {
      if (I > 0)
        OS << ", ";
      OS << "-1";
    }
    OS << "}";
    if (R != Rs - 1)
      OS << ",";
    OS << "\n";
  }
  OS << "  };\n";
}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeSwitch(
    std::map<unsigned, std::vector<std::string>> InstMap) const {
  OS << "  switch (Opcode) {\n";
  OS << "  default: return -1;\n";
  for (auto &P : InstMap) {
    auto OpMapIdx = P.first;
    const auto &Insts = P.second;
    for (const auto &Inst : Insts) {
      OS << "  case " << Inst << ":\n";
    }
    OS << "    return TypeMap[" << OpMapIdx << "][LogicalOpIdx];\n";
  }
  OS << "  }\n";
}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeReturn() const {
  OS << "  return -1;\n";
}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeEnd() const { OS << "}\n"; }

void PrinterCapstone::instrInfoEmitDeclareMCInstFeatureClasses() const {
  OS << "class MCInst;\n";
  OS << "class FeatureBitset;\n\n";
}

void PrinterCapstone::instrInfoEmitPredFcnDecl(
    RecVec const &TIIPredicates) const {
  for (const Record *Rec : TIIPredicates) {
    OS << "bool " << Rec->getValueAsString("FunctionName")
       << "(const MCInst &MI);\n";
  }

  OS << "void verifyInstructionPredicates(unsigned Opcode, const FeatureBitset "
        "&Features);\n";
}

void PrinterCapstone::instrInfoEmitPredFcnImpl(StringRef const &TargetName,
                                               RecVec const &TIIPredicates) {
  initNewPE(TargetName);
  PE->setExpandForMC(true);
  for (const Record *Rec : TIIPredicates) {
    OS << "bool " << Rec->getValueAsString("FunctionName");
    OS << "(const MCInst &MI) {\n";

    OS.indent(PE->getIndentLevel() * 2);
    PE->expandStatement(OS, Rec->getValueAsDef("Body"));
    OS << "\n}\n\n";
  }
}

void PrinterCapstone::instrInfoEmitInstrPredVerifierIncludes() const {
  OS << "#include <sstream>\n\n";
}

void PrinterCapstone::instrInfoEmitSubtargetFeatureBitEnumeration(
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> &SubtargetFeatures)
    const {
  // Emit the subtarget feature enumeration.
  SubtargetFeatureInfo::emitSubtargetFeatureBitEnumeration(SubtargetFeatures,
                                                           OS);
}

void PrinterCapstone::instrInfoEmitEmitSTFNameTable(
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> &SubtargetFeatures)
    const {
  OS << "#ifndef NDEBUG\n";
  SubtargetFeatureInfo::emitNameTable(SubtargetFeatures, OS);
  OS << "#endif // NDEBUG\n\n";
}

void PrinterCapstone::instrInfoEmitFeatureBitsEnum(
    std::vector<std::vector<Record *>> const &FeatureBitsets) const {
  OS << "// Feature bitsets.\n"
     << "enum : " << getMinimalTypeForRange(FeatureBitsets.size()) << " {\n"
     << "  CEFBS_None,\n";
  for (const auto &FeatureBitset : FeatureBitsets) {
    if (FeatureBitset.empty())
      continue;
    OS << "  " << getNameForFeatureBitset(FeatureBitset) << ",\n";
  }
  OS << "};\n\n";
}

void PrinterCapstone::instrInfoEmitFeatureBitsArray(
    std::vector<std::vector<Record *>> const &FeatureBitsets,
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> const
        &SubtargetFeatures) const {
  OS << "static constexpr FeatureBitset FeatureBitsets[] = {\n"
     << "  {}, // CEFBS_None\n";
  for (const auto &FeatureBitset : FeatureBitsets) {
    if (FeatureBitset.empty())
      continue;
    OS << "  {";
    for (const auto &Feature : FeatureBitset) {
      const auto &I = SubtargetFeatures.find(Feature);
      assert(I != SubtargetFeatures.end() && "Didn't import predicate?");
      OS << I->second.getEnumBitName() << ", ";
    }
    OS << "},\n";
  }
  OS << "};\n";
}

void PrinterCapstone::instrInfoEmitPredVerifier(
    std::vector<std::vector<Record *>> const &FeatureBitsets,
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> const
        &SubtargetFeatures,
    CodeGenTarget const &Target) const {
  OS << "void verifyInstructionPredicates(\n"
     << "    unsigned Opcode, const FeatureBitset &Features) {\n";
  emitIfNotDef("NDEBUG", true);
  OS << "  static " << getMinimalTypeForRange(FeatureBitsets.size())
     << " RequiredFeaturesRefs[] = {\n";
  unsigned InstIdx = 0;
  for (const CodeGenInstruction *Inst : Target.getInstructionsByEnumValue()) {
    OS << "    CEFBS";
    unsigned NumPredicates = 0;
    for (Record *Predicate : Inst->TheDef->getValueAsListOfDefs("Predicates")) {
      const auto &I = SubtargetFeatures.find(Predicate);
      if (I != SubtargetFeatures.end()) {
        OS << '_' << I->second.TheDef->getName();
        NumPredicates++;
      }
    }
    if (!NumPredicates)
      OS << "_None";
    OS << ", // " << Inst->TheDef->getName() << " = " << InstIdx << "\n";
    InstIdx++;
  }
  OS << "  };\n\n";
  OS << "  assert(Opcode < " << InstIdx << ");\n";
  OS << "  FeatureBitset AvailableFeatures = "
        "computeAvailableFeatures(Features);\n";
  OS << "  const FeatureBitset &RequiredFeatures = "
        "FeatureBitsets[RequiredFeaturesRefs[Opcode]];\n";
  OS << "  FeatureBitset MissingFeatures =\n"
     << "      (AvailableFeatures & RequiredFeatures) ^\n"
     << "      RequiredFeatures;\n"
     << "  if (MissingFeatures.any()) {\n"
     << "    std::ostringstream Msg;\n"
     << "    Msg << \"Attempting to emit \" << &" << Target.getName()
     << "InstrNameData[" << Target.getName() << "InstrNameIndices[Opcode]]\n"
     << "        << \" instruction but the \";\n"
     << "    for (unsigned i = 0, e = MissingFeatures.size(); i != e; ++i)\n"
     << "      if (MissingFeatures.test(i))\n"
     << "        Msg << SubtargetFeatureNames[i] << \" \";\n"
     << "    Msg << \"predicate(s) are not met\";\n"
     << "    report_fatal_error(Msg.str().c_str());\n"
     << "  }\n";
  emitIfNotDef("NDEBUG", false);
  OS << "}\n";
}

void PrinterCapstone::instrInfoEmitEnums(
    CodeGenTarget const &Target, StringRef const &Namespace,
    CodeGenSchedModels const &SchedModels) const {
  emitIncludeToggle("GET_INSTRINFO_ENUM", true);

  emitNamespace("llvm", true);
  // We must emit the PHI opcode first...
  emitNamespace(Namespace.str(), true);
  unsigned Num = 0;
  OS << "  enum {\n";
  for (const CodeGenInstruction *Inst : Target.getInstructionsByEnumValue())
    OS << "    " << Inst->TheDef->getName() << "\t= " << Num++ << ",\n";
  OS << "    INSTRUCTION_LIST_END = " << Num << "\n";
  OS << "  };\n\n";
  emitNamespace(Namespace.str(), false);
  emitNamespace("llvm", false);
  emitIncludeToggle("GET_INSTRINFO_ENUM", false);

  emitIncludeToggle("GET_INSTRINFO_SCHED_ENUM", true);
  emitNamespace("llvm", true);
  emitNamespace(Namespace.str(), true);
  emitNamespace("Sched", true);
  Num = 0;
  OS << "  enum {\n";
  for (const auto &Class : SchedModels.explicit_classes())
    OS << "    " << Class.Name << "\t= " << Num++ << ",\n";
  OS << "    SCHED_LIST_END = " << Num << "\n";
  OS << "  };\n";
  emitNamespace("Sched", false);
  emitNamespace(Namespace.str(), false);
  emitNamespace("llvm", false);

  emitIncludeToggle("GET_INSTRINFO_SCHED_ENUM", false);
}

void PrinterCapstone::instrInfoEmitTIIPredicates(StringRef const &TargetName,
                                                 RecVec const &TIIPredicates,
                                                 bool ExpandDefinition) {
  initNewPE(TargetName);
  PE->setExpandForMC(false);

  for (const Record *Rec : TIIPredicates) {
    instrInfoEmitTIIHelperMethod(TargetName, Rec, ExpandDefinition);
  }
}

void PrinterCapstone::instrInfoEmitComputeAssemblerAvailableFeatures(
    StringRef const &TargetName,
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> &SubtargetFeatures)
    const {
  SubtargetFeatureInfo::emitComputeAssemblerAvailableFeatures(
      TargetName, "", "computeAvailableFeatures", SubtargetFeatures, OS);
}

//--------------------------
// Backend: AsmMatcher
//--------------------------

static std::string getImplicitUses(StringRef const &TargetName,
                                   CodeGenInstruction const *Inst) {
  std::string Flags = "{ ";
  for (Record const *U : Inst->ImplicitUses) {
    assert(U->isSubClassOf("Register"));
    Flags += TargetName.str() + "_REG_" + U->getName().str() + ", ";
  }
  Flags += "0 }";
  return Flags;
}

static std::string getImplicitDefs(StringRef const &TargetName,
                                   CodeGenInstruction const *Inst) {
  std::string Flags = "{ ";
  for (Record const *U : Inst->ImplicitDefs) {
    assert(U->isSubClassOf("Register"));
    Flags += TargetName.str() + "_REG_" + U->getName().str() + ", ";
  }
  Flags += "0 }";
  return Flags;
}

static std::string getReqFeatures(StringRef const &TargetName,
                                  std::unique_ptr<MatchableInfo> const &MI) {
  std::string Flags = "{ ";
  CodeGenInstruction const *Inst = MI->getResultInst();
  std::string Mn = MI->Mnemonic.upper();
  // The debug if
  if (Inst->isBranch && !Inst->isCall) {
    Flags += TargetName.str() + "_GRP_JUMP, ";
  }
  if (Inst->isCall) {
    Flags += TargetName.str() + "_GRP_CALL, ";
  }
  for (const auto &OpInfo : Inst->Operands.OperandList) {
    if (OpInfo.OperandType == "MCOI::OPERAND_PCREL" &&
        (Inst->isBranch || Inst->isIndirectBranch || Inst->isCall)) {
      Flags += TargetName.str() + "_GRP_BRANCH_RELATIVE, ";
    }
  }
  // The group flags <ARCH>_GRP_PRIVILEGE and <ARCH>_GRP_INT (interrupt) are not
  // handled here. LLVM does not provide this info.
  for (SubtargetFeatureInfo const *STF : MI->RequiredFeatures) {
    Flags +=
        TargetName.str() + "_FEATURE_" + STF->TheDef->getName().str() + ", ";
  }
  Flags += "0 }";
  return Flags;
}

void PrinterCapstone::printInsnMapEntry(
    StringRef const &TargetName, std::unique_ptr<MatchableInfo> const &MI,
    raw_string_ostream &InsnMap) const {
  CodeGenInstruction const *Inst = MI->getResultInst();
  InsnMap << "{\n";
  InsnMap.indent(2) << TargetName << "_" << Inst->TheDef->getName();
  InsnMap << ", " << TargetName << "_INS_" << MI->Mnemonic.upper() << ",\n";
  InsnMap.indent(2) << "#ifndef CAPSTONE_DIET\n";
  InsnMap.indent(4) << getImplicitUses(TargetName, Inst) << ", ";
  InsnMap << getImplicitDefs(TargetName, Inst) << ", ";
  InsnMap << getReqFeatures(TargetName, MI) << ", ";
  InsnMap << (Inst->isBranch ? "1" : "0") << ", ";
  InsnMap << (Inst->isIndirectBranch ? "1" : "0") << "\n";
  InsnMap.indent(2) << "#endif\n";
  InsnMap << "},\n";
}

static std::string getCSAccess(short Access) {
  if (Access == 1)
    return "CS_AC_READ";
  else if (Access == 2)
    return "CS_AC_WRITE";
  else if (Access == 3)
    return "CS_AC_READ | CS_AC_WRITE";
  else
    PrintFatalNote("Invalid access flags set.");
}

static std::string getCSOperandType(Record const *OpRec) {
  std::string OperandType;
  if (OpRec->isSubClassOf("Operand") || OpRec->isSubClassOf("RegisterOperand"))
    OperandType = std::string(OpRec->getValueAsString("OperandType"));
  else if (OpRec->isSubClassOf("RegisterClass") ||
           OpRec->isSubClassOf("PointerLikeRegClass"))
    OperandType = "OPERAND_REGISTER";
  else
    return "";
  if (OperandType == "OPERAND_UNKNOWN") {
    if (OpRec->getValueAsDef("Type")->getValueAsInt("Size") == 0)
      // Pseudo type
      return "";
    OperandType = "OPERAND_IMMEDIATE";
  }
  if (OperandType == "OPERAND_PCREL" || OperandType == "OPERAND_IMMEDIATE")
    OperandType = "CS_OP_IMM";
  else if (OperandType == "OPERAND_MEMORY")
    OperandType = "CS_OP_MEM";
  else if (OperandType == "OPERAND_REGISTER")
    OperandType = "CS_OP_REG";
  // Arch dependent special Op types
  else if (OperandType == "OPERAND_VPRED_N" || OperandType == "OPERAND_VPRED_R")
    return "";
  else
    PrintFatalNote("Unhandled OperandType: " + OperandType);
  return OperandType;
}

void PrinterCapstone::printInsnOpMapEntry(
    CodeGenTarget const &Target, std::unique_ptr<MatchableInfo> const &MI,
    raw_string_ostream &InsnOpMap) const {

  typedef struct OpData {
    Record *Rec;
    std::string OpAsm;
    std::string OpType;
    unsigned
        Access; ///< 0b00 = unkown, 0b01 = In, 0b10 = Out, 0b11 = In and Out
    std::string str() const {
      return "Asm: " + OpAsm + " Type: " + OpType +
             " Access: " + std::to_string(Access);
    }
  } OpData;

  StringRef TargetName = Target.getName();
  CodeGenInstruction const *Inst = MI->getResultInst();
  DagInit *InDI = Inst->TheDef->getValueAsDag("InOperandList");
  DagInit *OutDI = Inst->TheDef->getValueAsDag("OutOperandList");
  unsigned NumDefs = OutDI->getNumArgs();

  // Check if op is in in or out operands
  unsigned E = InDI->getNumArgs() + OutDI->getNumArgs();
  bool isOutOp;
  std::vector<OpData> InsOps;
  // Interate over every In and Out operand and get its Def.
  for (unsigned I = 0; I != E; ++I) {
    Init *ArgInit;
    StringRef ArgName;
    isOutOp = I < NumDefs;
    if (isOutOp) {
      ArgInit = OutDI->getArg(I);
      ArgName = OutDI->getArgNameStr(I);
    } else {
      ArgInit = InDI->getArg(I - NumDefs);
      ArgName = InDI->getArgNameStr(I - NumDefs);
    }
    DagInit *SubArgDag = dyn_cast<DagInit>(ArgInit);
    if (SubArgDag)
      ArgInit = SubArgDag->getOperator();
    DefInit *Arg = dyn_cast<DefInit>(ArgInit);
    Record *Rec = Arg->getDef();

    // Determine Operand type
    std::string OperandType = getCSOperandType(Rec);
    if (OperandType == "")
      continue;

    // Check if Operand was already seen before (as In or Out operand).
    // If so update its access flags.
    bool OpExists = false;
    for (OpData &OD : InsOps) {
      if (OD.OpAsm == ArgName ||
          // ARM way of marking registers which are IN and OUT
          OD.OpAsm == (ArgName.str() + "_src") ||
          OD.OpAsm + "_src" == ArgName.str()) {
        OpExists = true;
        OD.Access |= isOutOp ? 2 : 1;
        break;
      }
    }
    if (!OpExists) {
      unsigned flag = isOutOp ? 2 : 1;
      OpData OD = {Rec, ArgName.str(), OperandType, flag};
      InsOps.emplace_back(OD);
    }
  }

  if (InsOps.size() > 7) {
    for (OpData const &OD : InsOps) {
      PrintNote(OD.str());
      OD.Rec->dump();
    }
    PrintFatalNote("Inst has more then 7 operands: " + Inst->AsmString);
  }
  // Write the C struct of the Instruction operands.
  InsnOpMap << "{ /* " + TargetName + "_INS_" + MI->Mnemonic.upper() + " - " +
                   Inst->TheDef->getName() + " */\n";
  for (OpData const &OD : InsOps) {
    InsnOpMap.indent(2) << "{ " + OD.OpType + ", " + getCSAccess(OD.Access) +
                               " }, /* " + OD.OpAsm + " */\n";
  }
  InsnOpMap.indent(2) << "{ 0 }\n";
  InsnOpMap << "},\n";
}

void PrinterCapstone::printInsnNameMapEnumEntry(
    StringRef const &TargetName, std::unique_ptr<MatchableInfo> const &MI,
    raw_string_ostream &InsnNameMap, raw_string_ostream &InsnEnum) const {
  static std::set<std::string> CSInsn;
  StringRef Mnemonic = MI->Mnemonic;
  if (CSInsn.find(Mnemonic.str()) != CSInsn.end())
    // Instruction already emitted.
    return;
  std::string EnumName = TargetName.str() + "_INS_" + Mnemonic.upper();
  CSInsn.emplace(Mnemonic);

  InsnNameMap.indent(2) << "\"" + Mnemonic + "\", // " + EnumName + "\n";
  InsnEnum.indent(2) << EnumName + ",\n";
}

void PrinterCapstone::printFeatureEnumEntry(
    StringRef const &TargetName, std::unique_ptr<MatchableInfo> const &MI,
    raw_string_ostream &FeatureEnum) const {
  static std::set<std::string> Features;
  
  for (SubtargetFeatureInfo const *STF : MI->RequiredFeatures) {
    std::string Feature = STF->TheDef->getName().str();
    if (Features.find(Feature) != Features.end())
      continue;
    Features.emplace(Feature);
    FeatureEnum.indent(2) << TargetName.str() + "_FEATURE_" + STF->TheDef->getName().str();
    if (Features.size() == 1)
      FeatureEnum << " = 128";
    FeatureEnum << ",\n";
  }
}

static void addHeader(raw_string_ostream &InsnMap,
                      raw_string_ostream &InsnOpMap,
                      raw_string_ostream &InsnNameMap,
                      raw_string_ostream &InsnEnum) {
  std::string HeaderComment =
      "/* Capstone Disassembly Engine, https://www.capstone-engine.org */\n"
      "/* By Nguyen Anh Quynh <aquynh@gmail.com>, 2013-2019 */\n"
      "/* By Rot127 <unisono@quyllur.org>, 2023 */\n"
      "\n"
      "/* Auto generated file. Do not edit. */\n"
      "/* Code generator: "
      "https://github.com/capstone-engine/capstone/tree/next/suite/auto-sync "
      "*/\n\n";

  InsnMap << HeaderComment;
  InsnOpMap << HeaderComment;
  InsnNameMap << HeaderComment;
  InsnEnum << HeaderComment;
}

void PrinterCapstone::writeFile(std::string Filename,
                                std::string const &Str) const {
  std::error_code EC;
  ToolOutputFile InsnMapFile(Filename, EC, sys::fs::OF_Text);
  if (EC)
    PrintFatalNote("Could no write \"" + Filename + "\" Error:\n" +
                   EC.message());
  InsnMapFile.os() << Str;
  InsnMapFile.keep();
}

/// This function emits all the mapping files and
/// Instruction enum for the current architecture.
void PrinterCapstone::asmMatcherEmitMatchTable(CodeGenTarget const &Target,
                                               AsmMatcherInfo const &Info,
                                               StringToOffsetTable &StringTable,
                                               unsigned VariantCount) const {
  std::string InsnMapStr;
  std::string InsnOpMapStr;
  std::string InsnNameMapStr;
  std::string InsnEnumStr;
  std::string FeatureEnumStr;
  raw_string_ostream InsnMap(InsnMapStr);
  raw_string_ostream InsnOpMap(InsnOpMapStr);
  raw_string_ostream InsnNameMap(InsnNameMapStr);
  raw_string_ostream InsnEnum(InsnEnumStr);
  raw_string_ostream FeatureEnum(FeatureEnumStr);
  addHeader(InsnMap, InsnOpMap, InsnNameMap, InsnEnum);

  // Currently we ignore any other Asm variant then the primary.
  Record *AsmVariant = Target.getAsmParserVariant(0);
  int AsmVariantNo = AsmVariant->getValueAsInt("Variant");

  for (const auto &MI : Info.Matchables) {
    if (MI->AsmVariantID != AsmVariantNo)
      continue;
    printInsnMapEntry(Target.getName(), MI, InsnMap);
    printInsnOpMapEntry(Target, MI, InsnOpMap);
    printInsnNameMapEnumEntry(Target.getName(), MI, InsnNameMap, InsnEnum);
    printFeatureEnumEntry(Target.getName(), MI, FeatureEnum);
  }

  std::string TName = Target.getName().str();
  std::string InsnMapFilename = TName + "MappingInsn.inc";
  writeFile(InsnMapFilename, InsnMapStr);
  InsnMapFilename = TName + "MappingInsnOp.inc";
  writeFile(InsnMapFilename, InsnOpMapStr);
  InsnMapFilename = TName + "MappingInsnName.inc";
  writeFile(InsnMapFilename, InsnNameMapStr);
  InsnMapFilename = TName + "InsnEnum.inc";
  writeFile(InsnMapFilename, InsnEnumStr);
  InsnMapFilename = TName + "FeatureEnum.inc";
  writeFile(InsnMapFilename, FeatureEnumStr);
}

void PrinterCapstone::asmMatcherEmitSourceFileHeader(
    std::string const &Desc) const {}
void PrinterCapstone::asmMatcherEmitDeclarations(bool HasOptionalOperands,
                                                 bool ReportMultipleNearMisses,
                                                 bool HasOperandInfos) const {}
void PrinterCapstone::asmMatcherEmitOperandDiagTypes(
    std::set<StringRef> const Types) const {}

void PrinterCapstone::asmMatcherEmitGetSubtargetFeatureName(
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> const
        SubtargetFeatures) const {}
void PrinterCapstone::asmMatcherEmitConversionFunctionI(
    StringRef const &TargetName, StringRef const &ClassName,
    std::string const &TargetOperandClass, bool HasOptionalOperands,
    size_t MaxNumOperands) const {}
void PrinterCapstone::asmMatcherEmitConversionFunctionII(
    std::string const &EnumName, StringRef const &AsmMatchConverter) const {}
void PrinterCapstone::asmMatcherEmitConversionFunctionIII(
    std::string const &EnumName, std::string const TargetOperandClass,
    bool HasOptionalOperands, MatchableInfo::AsmOperand const &Op,
    MatchableInfo::ResOperand const &OpInfo) const {}
void PrinterCapstone::asmMatcherEmitConversionFunctionIV(
    std::string const &EnumName, int64_t Val) const {}
void PrinterCapstone::asmMatcherEmitConversionFunctionV(
    std::string const &EnumName, std::string const &Reg) const {}
void PrinterCapstone::asmMatcherEmitConversionFunctionVI() const {}
void PrinterCapstone::asmMatcherWriteCvtOSToOS() const {}
void PrinterCapstone::asmMatcherEmitOperandFunctionI(
    StringRef const &TargetName, StringRef const &ClassName) const {}
void PrinterCapstone::asmMatcherEmitOperandFunctionII(
    std::string const &EnumName, MatchableInfo::AsmOperand const &Op,
    MatchableInfo::ResOperand const &OpInfo) const {}
void PrinterCapstone::asmMatcherEmitOperandFunctionIII(
    std::string const &EnumName) const {}
void PrinterCapstone::asmMatcherEmitOperandFunctionIV(
    std::string const &EnumName) const {}
void PrinterCapstone::asmMatcherEmitOperandFunctionV() const {}
void PrinterCapstone::asmMatcherEmitTiedOperandEnum(
    std::map<std::tuple<uint8_t, uint8_t, uint8_t>, std::string>
        TiedOperandsEnumMap) const {}
void PrinterCapstone::asmMatcherWriteOpOSToOS() const {}
void PrinterCapstone::asmMatcherEmitTiedOpTable(
    std::map<std::tuple<uint8_t, uint8_t, uint8_t>, std::string>
        TiedOperandsEnumMap) const {}
void PrinterCapstone::asmMatcherEmitTiedOpEmptyTable() const {}
void PrinterCapstone::asmMatcherEmitOperandConvKindEnum(
    SmallSetVector<CachedHashString, 16> OperandConversionKinds) const {}
void PrinterCapstone::asmMatcherEmitInstrConvKindEnum(
    SmallSetVector<CachedHashString, 16> InstructionConversionKinds) const {}
void PrinterCapstone::asmMatcherEmitConversionTable(
    size_t MaxRowLength,
    std::vector<std::vector<uint8_t>> const ConversionTable,
    SmallSetVector<CachedHashString, 16> InstructionConversionKinds,
    SmallSetVector<CachedHashString, 16> OperandConversionKinds,
    std::map<std::tuple<uint8_t, uint8_t, uint8_t>, std::string>
        TiedOperandsEnumMap) const {}
void PrinterCapstone::asmMatcherEmitMatchClassKindEnum(
    std::forward_list<ClassInfo> const &Infos) const {}
void PrinterCapstone::asmMatcherEmitMatchClassDiagStrings(
    AsmMatcherInfo const &Info) const {}
void PrinterCapstone::asmMatcherEmitRegisterMatchErrorFunc(
    AsmMatcherInfo &Info) const {}
void PrinterCapstone::asmMatcherEmitIsSubclassI() const {}
bool PrinterCapstone::asmMatcherEmitIsSubclassII(
    bool EmittedSwitch, std::string const &Name) const {
  return true;
}
void PrinterCapstone::asmMatcherEmitIsSubclassIII(StringRef const &Name) const {
}
void PrinterCapstone::asmMatcherEmitIsSubclassIV(
    std::vector<StringRef> const &SuperClasses) const {}
void PrinterCapstone::asmMatcherEmitIsSubclassV(bool EmittedSwitch) const {}
void PrinterCapstone::asmMatcherEmitValidateOperandClass(
    AsmMatcherInfo &Info) const {}
void PrinterCapstone::asmMatcherEmitMatchClassKindNames(
    std::forward_list<ClassInfo> &Infos) const {}
void PrinterCapstone::asmMatcherEmitAsmTiedOperandConstraints(
    CodeGenTarget &Target, AsmMatcherInfo &Info) const {}
std::string PrinterCapstone::getNameForFeatureBitset(
    const std::vector<Record *> &FeatureBitset) const {
  return "";
}
void PrinterCapstone::asmMatcherEmitFeatureBitsetEnum(
    std::vector<std::vector<Record *>> const FeatureBitsets) const {}
void PrinterCapstone::asmMatcherEmitFeatureBitsets(
    std::vector<std::vector<Record *>> const FeatureBitsets,
    AsmMatcherInfo const &Info) const {}
void PrinterCapstone::asmMatcherEmitMatchEntryStruct(
    unsigned MaxMnemonicIndex, unsigned NumConverters, size_t MaxNumOperands,
    std::vector<std::vector<Record *>> const FeatureBitsets,
    AsmMatcherInfo const &Info) const {}
void PrinterCapstone::asmMatcherEmitMatchFunction(
    CodeGenTarget const &Target, Record const *AsmParser,
    StringRef const &ClassName, bool HasMnemonicFirst, bool HasOptionalOperands,
    bool ReportMultipleNearMisses, bool HasMnemonicAliases,
    size_t MaxNumOperands, bool HasDeprecation,
    unsigned int VariantCount) const {}
void PrinterCapstone::asmMatcherEmitMnemonicSpellChecker(
    CodeGenTarget const &Target, unsigned VariantCount) const {}
void PrinterCapstone::asmMatcherEmitMnemonicChecker(
    CodeGenTarget const &Target, unsigned VariantCount, bool HasMnemonicFirst,
    bool HasMnemonicAliases) const {}
void PrinterCapstone::asmMatcherEmitCustomOperandParsing(
    unsigned MaxMask, CodeGenTarget &Target, AsmMatcherInfo const &Info,
    StringRef ClassName, StringToOffsetTable &StringTable,
    unsigned MaxMnemonicIndex, unsigned MaxFeaturesIndex, bool HasMnemonicFirst,
    Record const &AsmParser) const {}
void PrinterCapstone::asmMatcherEmitIncludes() const {}
void PrinterCapstone::asmMatcherEmitMnemonicTable(
    StringToOffsetTable &StringTable) const {}
void PrinterCapstone::asmMatcherEmitMatchRegisterName(
    Record const *AsmParser,
    std::vector<StringMatcher::StringPair> const Matches) const {}
void PrinterCapstone::asmMatcherEmitMatchTokenString(
    std::vector<StringMatcher::StringPair> const Matches) const {}
void PrinterCapstone::asmMatcherEmitMatchRegisterAltName(
    Record const *AsmParser,
    std::vector<StringMatcher::StringPair> const Matches) const {}
void PrinterCapstone::asmMatcherEmitMnemonicAliasVariant(
    std::vector<StringMatcher::StringPair> const &Cases,
    unsigned Indent) const {}
void PrinterCapstone::asmMatcherAppendMnemonicAlias(
    Record const *R, std::string const &FeatureMask,
    std::string &MatchCode) const {}
void PrinterCapstone::asmMatcherAppendMnemonic(Record const *R,
                                               std::string &MatchCode) const {}
void PrinterCapstone::asmMatcherAppendMnemonicAliasEnd(
    std::string &MatchCode) const {}
void PrinterCapstone::asmMatcherEmitApplyMnemonicAliasesI() const {}
void PrinterCapstone::asmMatcherEmitApplyMnemonicAliasesII(
    int AsmParserVariantNo) const {}
void PrinterCapstone::asmMatcherEmitApplyMnemonicAliasesIII() const {}
void PrinterCapstone::asmMatcherEmitApplyMnemonicAliasesIV() const {}
void PrinterCapstone::asmMatcherEmitApplyMnemonicAliasesV() const {}
void PrinterCapstone::asmMatcherEmitSTFBitEnum(AsmMatcherInfo &Info) const {}
void PrinterCapstone::asmMatcherEmitComputeAssemblerAvailableFeatures(
    AsmMatcherInfo &Info, StringRef const &ClassName) const {}
} // end namespace llvm
