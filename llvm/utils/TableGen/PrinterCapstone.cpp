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
#include "llvm/MC/MCInst.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/FormatVariadic.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/TableGen/TableGenBackend.h"
#include <algorithm>
#include <regex>
#include <unordered_map>

static void emitDefaultSourceFileHeader(formatted_raw_ostream &OS) {
  OS << "/* Capstone Disassembly Engine, http://www.capstone-engine.org */\n";
  OS << "/* By Nguyen Anh Quynh <aquynh@gmail.com>, 2013-2022, */\n";
  OS << "/*    Rot127 <unisono@quyllur.org> 2022-2023 */\n";
  OS << "/* Automatically generated file by the LLVM TableGen Disassembler "
        "Backend. */\n";
  OS << "/* Do not edit. */\n\n";
}

static void addHeader(raw_string_ostream &Stream) {
  std::string HeaderComment =
      "/* Capstone Disassembly Engine, https://www.capstone-engine.org */\n"
      "/* By Nguyen Anh Quynh <aquynh@gmail.com>, 2013-2019 */\n"
      "/* By Rot127 <unisono@quyllur.org>, 2023 */\n"
      "\n"
      "/* Auto generated file. Do not edit. */\n"
      "/* Code generator: "
      "https://github.com/capstone-engine/capstone/tree/next/suite/auto-sync "
      "*/\n\n";

  Stream << HeaderComment;
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
  std::set<std::string> Ignore = {"GET_REGINFO_TARGET_DESC",
                                  "GET_REGINFO_HEADER",
                                  "GET_MNEMONIC_CHECKER",
                                  "GET_MNEMONIC_SPELL_CHECKER",
                                  "GET_MATCHER_IMPLEMENTATION",
                                  "GET_SUBTARGET_FEATURE_NAME",
                                  "GET_REGISTER_MATCHER",
                                  "GET_OPERAND_DIAGNOSTIC_TYPES",
                                  "GET_ASSEMBLER_HEADER",
                                  "GET_INSTRINFO_HEADER",
                                  "GET_INSTRINFO_HELPER_DECLS",
                                  "GET_INSTRINFO_HELPERS",
                                  "GET_INSTRINFO_CTOR_DTOR",
                                  "GET_INSTRINFO_OPERAND_ENUM",
                                  "GET_INSTRINFO_NAMED_OPS",
                                  "GET_INSTRINFO_OPERAND_TYPES_ENUM",
                                  "GET_INSTRINFO_OPERAND_TYPE",
                                  "GET_INSTRINFO_MEM_OPERAND_SIZE",
                                  "GET_INSTRINFO_LOGICAL_OPERAND_SIZE_MAP",
                                  "GET_INSTRINFO_LOGICAL_OPERAND_TYPE_MAP",
                                  "GET_INSTRINFO_MC_HELPER_DECLS",
                                  "GET_INSTRINFO_MC_HELPERS",
                                  "ENABLE_INSTR_PREDICATE_VERIFIER",
                                  "GET_SUBTARGETINFO_MC_DESC",
                                  "GET_SUBTARGETINFO_TARGET_DESC",
                                  "GET_SUBTARGETINFO_HEADER",
                                  "GET_SUBTARGETINFO_CTOR",
                                  "GET_STIPREDICATE_DECLS_FOR_MC_ANALYSIS",
                                  "GET_STIPREDICATE_DEFS_FOR_MC_ANALYSIS",
                                  "GET_SUBTARGETINFO_MACRO"};
  if (Ignore.find(Name) != Ignore.end())
    return;
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

void PrinterCapstone::emitIfNotDef(std::string const &Name, bool Begin) const {
  if (Name == "NDEBUG")
    return;
  if (Begin) {
    OS << "#ifndef " << Name << "\n";
  } else {
    OS << "#endif // " << Name << "\n\n";
  }
}

void PrinterCapstone::regInfoEmitSourceFileHeader(
    std::string const &Desc) const {
  static unsigned Count = 0;
  if (Count > 0) {
    // Only emit it once at the beginning.
    return;
  }
  emitDefaultSourceFileHeader(OS);
  ++Count;
}

// runEnums - Print out enum values for all of the registers.
void PrinterCapstone::regInfoEmitEnums(CodeGenTarget const &Target,
                                       CodeGenRegBank const &Bank) const {
  std::string CSRegEnumStr;
  raw_string_ostream CSRegEnum(CSRegEnumStr);
  addHeader(CSRegEnum);

  const auto &Registers = Bank.getRegisters();

  // Register enums are stored as uint16_t in the tables. Make sure we'll fit.
  assert(Registers.size() <= 0xffff && "Too many regs to fit in tables");

  emitIncludeToggle("GET_REGINFO_ENUM", true);
  std::string const TargetName = Target.getName().str();

  OS << "enum {\n  " << TargetName << "_NoRegister,\n";
  CSRegEnum << "\t" << TargetName << "_REG_INVALID = 0,\n";

  for (const auto &Reg : Registers) {
    OS << "  " << TargetName << "_" << Reg.getName() << " = " << Reg.EnumValue
       << ",\n";
    CSRegEnum << "\t" << TargetName << "_REG_" << Reg.getName() << " = "
              << Reg.EnumValue << ",\n";
  }
  assert(Registers.size() == Registers.back().EnumValue &&
         "Register enum value mismatch!");
  OS << "  NUM_TARGET_REGS // " << Registers.size() + 1 << "\n";
  OS << "};\n";
  CSRegEnum << "\tARM_REG_ENDING, // " << Registers.size() + 1 << "\n";
  writeFile(TargetName + "GenCSRegEnum.inc", CSRegEnumStr);

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
    // For Capstone we are only interested in:
    // RegClass Name, Size group and bit size
    OS << "  { " << RCName << ", " << RCBitsName << ", " << RCBitsSize
       << " },\n";
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

void PrinterCapstone::regInfoEmitIsConstantPhysReg(
    std::deque<CodeGenRegister> const &Regs,
    std::string const &ClassName) const {}

static std::string resolveTemplateCall(std::string const &Dec) {
  unsigned long const B = Dec.find_first_of("<");
  unsigned long const E = Dec.find(">");
  if (B == std::string::npos) {
    return Dec;
  }
  std::string const &DecName = Dec.substr(0, B);
  std::string Args = Dec.substr(B + 1, E - B - 1);
  Args = std::regex_replace(Args, std::regex("true"), "1");
  Args = std::regex_replace(Args, std::regex("false"), "0");
  std::string Decoder =
      DecName + "_" + std::regex_replace(Args, std::regex("\\s*,\\s*"), "_");
  return Decoder;
}

void PrinterCapstone::decoderEmitterEmitOpDecoder(raw_ostream &DecoderOS,
                                                  const OperandInfo &Op) const {
  unsigned const Indent = 4;
  DecoderOS.indent(Indent) << GuardPrefix;
  if (Op.Decoder.find("<") != std::string::npos) {
    DecoderOS << resolveTemplateCall(Op.Decoder);
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
                                   ? resolveTemplateCall(OpInfo.Decoder)
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
     << "      bool Pred = checkDecoderPredicate(MI, PIdx); \\\n"
     << "      if (!Pred) \\\n"
     << "        Ptr += NumToSkip; \\\n"
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
  OS.indent(Indentation)
      << "static bool checkDecoderPredicate(MCInst *Inst, unsigned Idx"
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
  OS << "#include <capstone/platform.h>\n"
     << "#include <assert.h>\n\n";
}

void PrinterCapstone::asmWriterEmitGetMnemonic(
    std::string const &TargetName, StringRef const &ClassName) const {
  OS << "/// getMnemonic - This method is automatically generated by "
        "tablegen\n"
        "/// from the instruction set description.\n"
        "MnemonicBitsInfo getMnemonic(MCInst *MI, SStream *O) {\n";
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
    BitsOS << "OpInfo" << Table << "[MCInst_getOpcode(MI)] << " << Shift
           << ";\n";
    // Prepare the shift for the next iteration and increment the table count.
    Shift += TableSize;
    ++Table;
  }

  OS << "  // Emit the opcode for the instruction.\n";
  OS << BitsString;

  // Return mnemonic string and bits.
  OS << "  MnemonicBitsInfo MBI = {AsmStrs+(Bits & " << (1 << AsmStrBits) - 1
     << ")-1, Bits};\n";
  OS << "  return MBI;\n\n";

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
        "void "
     << "printInstruction(MCInst *MI, uint64_t Address, "
     << "SStream *O) {\n";

  // Emit the initial tab character.
  OS << "  SStream_concat0(O, \"\");\n";

  // Emit the starting string.
  OS << "  MnemonicBitsInfo MnemonicInfo = getMnemonic(MI, O);\n\n";
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
         << "  default: assert(0 && \"Invalid command number.\");\n";

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
  OS << "  switch (MCInst_getOpcode(MI)) {\n";
  OS << "  default: assert(0 && \"Unexpected opcode.\");\n";
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
  OS << "  case " << FirstInst.CGI->Namespace << "_"
     << FirstInst.CGI->TheDef->getName() << ":\n";
  for (const AsmWriterInst &AWI : SimilarInsts)
    OS << "  case " << AWI.CGI->Namespace << "_" << AWI.CGI->TheDef->getName()
       << ":\n";
  for (unsigned I = 0, E = FirstInst.Operands.size(); I != E; ++I) {
    if (I != DifferingOperand) {
      // If the operand is the same for all instructions, just print it.
      OS << "    " << FirstInst.Operands[I].getCode(PassSubtarget);
    } else {
      // If this is the operand that varies between all of the instructions,
      // emit a switch for just this operand now.
      OS << "    switch (MCInst_getOpcode(MI)) {\n";
      OS << "    default: assert(0 && \"Unexpected opcode.\");\n";
      std::vector<std::pair<std::string, AsmWriterOperand>> OpsToPrint;
      OpsToPrint.push_back(
          std::make_pair(FirstInst.CGI->Namespace.str() + "_" +
                             FirstInst.CGI->TheDef->getName().str(),
                         FirstInst.Operands[I]));

      for (const AsmWriterInst &AWI : SimilarInsts) {
        OpsToPrint.push_back(std::make_pair(
            AWI.CGI->Namespace.str() + "_" + AWI.CGI->TheDef->getName().str(),
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
        "const char *";
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
       << "  default: assert(0 && \"Invalid register alt name "
          "index!\");\n";
    for (const Record *R : AltNameIndices) {
      StringRef const AltName = R->getName();
      OS << "  case ";
      if (!Namespace.empty())
        OS << Namespace << "_";
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
          OS << Namespace << "_";
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
  return "AliasPatternCond_K_Ignore, 0";
}

char const *PrinterCapstone::asmWriterGetPatCondKRegClass() const {
  return "AliasPatternCond_K_RegClass, {0}_{1}RegClassID";
}

char const *PrinterCapstone::asmWriterGetPatCondKTiedReg() const {
  return "AliasPatternCond_K_TiedReg, {0}";
}

char const *PrinterCapstone::asmWriterGetPatCondKCustom() const {
  return "AliasPatternCond_K_Custom, {0}";
}

char const *PrinterCapstone::asmWriterGetPatCondKImm() const {
  return "AliasPatternCond_K_Imm, (uint32_t){0}";
}

char const *PrinterCapstone::asmWriterGetPatCondKNoReg() const {
  return "AliasPatternCond_K_Reg, {0}_NoRegister";
}

char const *PrinterCapstone::asmWriterGetPatCondKReg() const {
  return "AliasPatternCond_K_Reg, {0}_{1}";
}

char const *PrinterCapstone::asmWriterGetPatCondKFeature() const {
  return "AliasPatternCond_K_{0}{1}Feature, {2}_{3}";
}

char const *PrinterCapstone::asmWriterGetPatCondKEndOrFeature() const {
  return "AliasPatternCond_K_EndOrFeatures, 0";
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
  OS << "bool printAliasInstr(MCInst"
     << " *MI, uint64_t Address, "
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
  OS.indent(2) << "{0},"; // Null terminated to ease binary search.
  OS.indent(2) << "};\n\n";
  OS.indent(2) << "static const AliasPattern Patterns[] = {\n";
  OS << PatternO.str();
  OS.indent(2) << "{0},";
  OS.indent(2) << "};\n\n";
  OS.indent(2) << "static const AliasPatternCond Conds[] = {\n";
  OS << CondO.str();
  OS.indent(2) << "{0},";
  OS.indent(2) << "};\n\n";
  OS.indent(2) << "static const char AsmStrings[] =\n";
  for (const auto &P : AsmStrings) {
    OS.indent(4) << "/* " << P.first << " */ \"" << P.second << "\\0\"\n";
  }

  OS.indent(2) << ";\n\n";

  // Assert that the opcode table is sorted. Use a static local constructor to
  // ensure that the check only happens once on first run.
  OS << "#ifndef NDEBUG\n";
  OS.indent(2) << "//static struct SortCheck {\n";
  OS.indent(2) << "//  SortCheck(ArrayRef<PatternsForOpcode> OpToPatterns) {\n";
  OS.indent(2) << "//    assert(std::is_sorted(\n";
  OS.indent(2)
      << "//               OpToPatterns.begin(), OpToPatterns.end(),\n";
  OS.indent(2) << "//               [](const PatternsForOpcode &L, const "
                  "//PatternsForOpcode &R) {\n";
  OS.indent(2) << "//                 return L.Opcode < R.Opcode;\n";
  OS.indent(2) << "//               }) &&\n";
  OS.indent(2)
      << "//           \"tablegen failed to sort opcode patterns\");\n";
  OS.indent(2) << "//  }\n";
  OS.indent(2) << "//} sortCheckVar(OpToPatterns);\n";
  OS << "#endif\n\n";

  OS.indent(2) << "AliasMatchingData M = {\n";
  OS.indent(2) << "  OpToPatterns,\n";
  OS.indent(2) << "  Patterns,\n";
  OS.indent(2) << "  Conds,\n";
  OS.indent(2) << "  AsmStrings,\n";
  OS.indent(2) << "};\n";

  OS.indent(2) << "const char *AsmString = matchAliasPatterns(MI, &M);\n";
  OS.indent(2) << "if (!AsmString) return false;\n\n";

  // Code that prints the alias, replacing the operands with the ones from the
  // MCInst.
  OS << "  unsigned I = 0;\n";
  OS << "  while (AsmString[I] != ' ' && AsmString[I] != '\\t' &&\n";
  OS << "         AsmString[I] != '$' && AsmString[I] != '\\0')\n";
  OS << "    ++I;\n";
  OS << "  SStream_concat1(OS, '\\t');\n"
     << "  char substr[I+1];\n"
     << "  memcpy(substr, AsmString, I);\n"
     << "  substr[I] = '\\0';\n"
     << "  SStream_concat0(OS, substr);\n";

  OS << "  if (AsmString[I] != '\\0') {\n";
  OS << "    if (AsmString[I] == ' ' || AsmString[I] == '\\t') {\n";
  OS << "      SStream_concat1(OS, '\\t');\n";
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
  OS << "OS);\n";
  OS << "        } else\n";
  OS << "          printOperand(MI, ((unsigned)AsmString[I++]) - 1, ";
  OS << "OS);\n";
  OS << "      } else {\n";
  OS << "        SStream_concat1(OS, AsmString[I++]);\n";
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
  OS << "void printCustomAliasOperand(\n"
     << "         MCInst *MI, uint64_t Address, unsigned OpIdx,\n"
     << "         unsigned PrintMethodIdx,\n"
     << "         SStream *OS) {\n";
  if (PrintMethods.empty())
    OS << "  llvm_unreachable(\"Unknown PrintMethod kind\");\n";
  else {
    OS << "  switch (PrintMethodIdx) {\n"
       << "  default:\n"
       << "    assert(0 && \"Unknown PrintMethod kind\");\n"
       << "    break;\n";

    for (unsigned I = 0; I < PrintMethods.size(); ++I) {
      OS << "  case " << I << ":\n"
         << "    " << PrintMethods[I].first << "(MI, "
         << (PrintMethods[I].second ? "Address, " : "") << "OpIdx, "
         << "OS);\n"
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
       << "                  unsigned PredicateIndex) {\n"
       << "  switch (PredicateIndex) {\n"
       << "  default:\n"
       << "    assert(0 && \"Unknown MCOperandPredicate kind\");\n"
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
  emitDefaultSourceFileHeader(OS);
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
    OS << "  " << TargetName << "_" << Def->getName() << " = " << I << ",\n";

    // Save the index for this feature.
    FeatureMap[Def] = I;
  }

  OS << "  " << TargetName << "_"
     << "NumSubtargetFeatures = " << N << "\n";

  // Close enumeration and namespace
  OS << "};\n";
}

void PrinterCapstone::subtargetEmitGetSTIMacro(
    StringRef const &Value, StringRef const &Attribute) const {}

void PrinterCapstone::subtargetEmitHwModes(CodeGenHwModes const &CGH,
                                           std::string const &ClassName) const {
}

void PrinterCapstone::subtargetEmitFeatureKVHeader(
    std::string const &Target) const {
  // Begin feature table
}

void PrinterCapstone::subtargetEmitFeatureKVPartI(
    std::string const &Target, StringRef const &CommandLineName,
    StringRef const &Name, StringRef const &Desc) const {}

void PrinterCapstone::subtargetEmitFeatureKVPartII() const {}

void PrinterCapstone::subtargetEmitPrintFeatureMask(
    std::array<uint64_t, MAX_SUBTARGET_WORDS> const &Mask) const {}

void PrinterCapstone::subtargetEmitFeatureKVEnd() const {}

void PrinterCapstone::subtargetEmitCPUKVHeader(
    std::string const &Target) const {}

void PrinterCapstone::subtargetEmitCPUKVEnd() const {}

void PrinterCapstone::subtargetEmitCPUKVPartI(StringRef const &Name) const {}

void PrinterCapstone::subtargetEmitCPUKVPartII() const {}

void PrinterCapstone::subtargetEmitCPUKVPartIII(
    std::string const &ProcModelName) const {}

void PrinterCapstone::subtargetEmitDBGMacrosBegin() const {}

void PrinterCapstone::subtargetEmitDBGMacrosEnd() const {}

void PrinterCapstone::subtargetEmitFunctionalItinaryUnits(
    CodeGenSchedModels const &SchedModels) const {}

std::string const PrinterCapstone::subtargetGetBeginStageTable(
    std::string const &TargetName) const {
  return "";
}

std::string const PrinterCapstone::subtargetGetBeginOperandCycleTable(
    std::string const &TargetName) const {
  return "";
}

std::string const PrinterCapstone::subtargetGetBeginBypassTable(
    std::string const &TargetName) const {
  return "";
}

std::string const PrinterCapstone::subtargetGetEndStageTable() const {
  return "";
}

std::string const PrinterCapstone::subtargetGetEndOperandCycleTable() const {
  return "";
}

std::string const PrinterCapstone::subtargetGetEndBypassTable() const {
  return "";
}

// subtargetFormItineraryStageString - Compose a string containing the stage
// data initialization for the specified itinerary.  N is the number
// of stages.
void PrinterCapstone::subtargetFormItineraryStageString(
    std::string const &Name, Record *ItinData, std::string &ItinString,
    unsigned &NStages) const {}

// FormItineraryOperandCycleString - Compose a string containing the
// operand cycle initialization for the specified itinerary.  N is the
// number of operands that has cycles specified.
void PrinterCapstone::subtargetFormItineraryOperandCycleString(
    Record *ItinData, std::string &ItinString, unsigned &NOperandCycles) const {
}

void PrinterCapstone::subtargetFormItineraryBypassString(
    const std::string &Name, Record *ItinData, std::string &ItinString,
    unsigned NOperandCycles) const {}

std::string
PrinterCapstone::subtargetGetStageEntryPartI(std::string const &ItinStageString,
                                             unsigned StageCount) const {
  return "";
}
std::string
PrinterCapstone::subtargetGetStageEntryPartII(unsigned StageCount,
                                              unsigned NStages) const {
  return "";
}
std::string PrinterCapstone::subtargetGetStageEntryPartIII() const {
  return "";
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartI(
    std::string const &ItinOperandCycleString) const {
  return "";
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartII(
    unsigned OperandCycleCount, unsigned NOperandCycles) const {
  return "";
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartIII(
    std::string const &OperandIdxComment) const {
  return "";
}

std::string PrinterCapstone::subtargetGetOperandCycleEntryPartIV(
    std::string const &ItinBypassString,
    std::string const &OperandIdxComment) const {
  return "";
}

void PrinterCapstone::subtargetEmitProcessorItineraryTable(
    std::string const &ItinsDefName, std::vector<InstrItinerary> &ItinList,
    CodeGenSchedModels const &SchedModels) const {}

void PrinterCapstone::subtargetEmitPreOperandTableComment() const {}

// Emit SchedClass tables for all processors and associated global tables.
void PrinterCapstone::subtargetEmitSchedClassTables(
    SchedClassTablesT &SchedTables, std::string const &TargetName,
    CodeGenSchedModels const &SchedModels) const {}

unsigned PrinterCapstone::subtargetEmitRegisterFileTables(
    CodeGenProcModel const &ProcModel) const {
  return 0;
}

void PrinterCapstone::subtargetEmitMCExtraProcInfoTableHeader(
    std::string const &ProcModelName) const {}

void PrinterCapstone::subtargetEmitMCExtraProcInfoTableEnd() const {}

void PrinterCapstone::subtargetEmitReorderBufferSize(
    int64_t ReorderBufferSize) const {}

void PrinterCapstone::subtargetEmitMaxRetirePerCycle(
    int64_t MaxRetirePerCycle) const {}

void PrinterCapstone::subtargetEmitRegisterFileInfo(
    CodeGenProcModel const &ProcModel, unsigned NumRegisterFiles,
    unsigned NumCostEntries) const {}

void PrinterCapstone::subtargetEmitResourceDescriptorLoadQueue(
    unsigned QueueID) const {}

void PrinterCapstone::subtargetEmitResourceDescriptorStoreQueue(
    unsigned QueueID) const {}

void PrinterCapstone::subtargetEmitProcessorResourceSubUnits(
    const CodeGenProcModel &ProcModel,
    CodeGenSchedModels const &SchedModels) const {}

void PrinterCapstone::subtargetEmitMCProcResourceDescHeader(
    std::string const &ProcModelName) const {}

void PrinterCapstone::subtargetEmitMCProcResourceDescEnd() const {}

void PrinterCapstone::subtargetEmitMCProcResourceDesc(
    Record const *PRDef, Record const *SuperDef,
    std::string const &ProcModelName, unsigned SubUnitsOffset,
    unsigned SuperIdx, unsigned NumUnits, int BufferSize, unsigned I,
    unsigned const SubUnitsBeginOffset) const {}

// Emit either the value defined in the TableGen Record, or the default
// value defined in the C++ header. The Record is null if the processor does not
// define a model.
void PrinterCapstone::subtargetEmitProcessorProp(Record const *R,
                                                 StringRef const Name,
                                                 char Separator) const {}

void PrinterCapstone::subtargetEmitProcModelHeader(
    std::string const &ModelName) const {}

void PrinterCapstone::subtargetEmitProcModel(
    CodeGenProcModel const &PM, CodeGenSchedModels const &SchedModels) const {}

void PrinterCapstone::subtargetEmitResolveVariantSchedClassImplHdr() const {}

void PrinterCapstone::subtargetEmitResolveVariantSchedClassImplEnd() const {}

void PrinterCapstone::subtargetEmitSchedClassSwitch() const {}

void PrinterCapstone::subtargetEmitSchedClassCase(
    unsigned VC, std::string const &SCName) const {}

void PrinterCapstone::subtargetEmitSchedClassProcGuard(
    unsigned Pi, bool OnlyExpandMCInstPredicates,
    std::string const &ModelName) const {}

// Indent <= -1 (default = -1) means previous PE indent level.
void PrinterCapstone::subtargetEmitPredicates(
    CodeGenSchedTransition const &T, CodeGenSchedClass const &SC,
    bool (*IsTruePredicate)(Record const *Rec), int Indent) const {}

void PrinterCapstone::subtargetEmitProcTransitionEnd() const {}

void PrinterCapstone::subtargetEmitSchedClassCaseEnd(
    CodeGenSchedClass const &SC) const {}

void PrinterCapstone::subtargetEmitSchedClassSwitchEnd() const {}

// Used by method `SubtargetEmitter::emitSchedModelHelpersImpl()` to generate
// epilogue code for the auto-generated helper.
void PrinterCapstone::subtargetEmitSchedModelHelperEpilogue(
    bool ShouldReturnZero) const {}

void PrinterCapstone::subtargetEmitGenMCSubtargetInfoClass(
    std::string const &TargetName, bool OverrideGetHwMode) const {}

void PrinterCapstone::subtargetEmitMCSubtargetInfoImpl(
    std::string const &TargetName, unsigned NumFeatures, unsigned NumProcs,
    bool SchedModelHasItin) const {}

void PrinterCapstone::subtargetEmitIncludeSTIDesc() const {}

void PrinterCapstone::subtargetEmitDFAPacketizerClass(
    std::string const &TargetName, std::string const &ClassName,
    bool OverrideGetHwMode) const {}

void PrinterCapstone::subtargetEmitDFASubtargetInfoImpl(
    std::string const &TargetName, std::string const &ClassName,
    unsigned NumFeatures, unsigned NumProcs, bool SchedModelHasItin) const {}

void PrinterCapstone::subtargetEmitDFAPacketizerClassEnd() const {}

void PrinterCapstone::subtargetEmitSTICtor() const {}

void PrinterCapstone::subtargetEmitExternKVArrays(
    std::string const &TargetName, bool SchedModelsHasItin) const {}

void PrinterCapstone::subtargetEmitClassDefs(std::string const &TargetName,
                                             std::string const &ClassName,
                                             unsigned NumFeatures,
                                             unsigned NumProcs,
                                             bool SchedModelsHasItin) const {}

void PrinterCapstone::subtargetEmitResolveSchedClassHdr(
    std::string const &ClassName) const {}

void PrinterCapstone::subtargetEmitResolveSchedClassEnd(
    std::string const &ClassName) const {}

void PrinterCapstone::subtargetEmitResolveVariantSchedClass(
    std::string const &TargetName, std::string const &ClassName) const {}

void PrinterCapstone::subtargetEmitPredicateProlog(
    const RecordKeeper &Records) const {}

void PrinterCapstone::subtargetEmitParseFeaturesFunction(
    std::string const &TargetName,
    std::vector<Record *> const &Features) const {}

void PrinterCapstone::subtargetEmitExpandedSTIPreds(
    StringRef const &TargetName, std::string const &ClassName,
    CodeGenSchedModels const &SchedModels) {}

void PrinterCapstone::subtargetPrepareSchedClassPreds(
    StringRef const &TargetName, bool OnlyExpandMCInstPredicates) {}

void PrinterCapstone::subtargetEmitExpandedSTIPredsMCAnaDecl(
    StringRef const &TargetName, CodeGenSchedModels const &SchedModels) {}

void PrinterCapstone::subtargetEmitExpandedSTIPredsMCAnaDefs(
    StringRef const &TargetName, std::string const &ClassPrefix,
    CodeGenSchedModels const &SchedModels) const {}

void PrinterCapstone::subtargetEmitExpandedSTIPredsHeader(
    StringRef const &TargetName, CodeGenSchedModels const &SchedModels) {}

void PrinterCapstone::subtargetEmitStageAndSycleTables(
    std::string const &StageTable, std::string const &OperandCycleTable,
    std::string const &BypassTable) const {}

//---------------------------
// Backend: InstrInfoEmitter
//---------------------------

void PrinterCapstone::instrInfoEmitSourceFileHeader() const {
  emitDefaultSourceFileHeader(OS);
}

void PrinterCapstone::instrInfoSetOperandInfoStr(
    std::string &Res, Record const *OpR, CGIOperandList::OperandInfo const &Op,
    CGIOperandList::ConstraintInfo const &Constraint) const {
  if (OpR->isSubClassOf("RegisterOperand"))
    OpR = OpR->getValueAsDef("RegClass");
  if (OpR->isSubClassOf("RegisterClass"))
    Res += OpR->getValueAsString("Namespace").str() + "_" +
           OpR->getName().str() + "RegClassID, ";
  else if (OpR->isSubClassOf("PointerLikeRegClass"))
    Res += utostr(OpR->getValueAsInt("RegClassKind")) + ", ";
  else
    // -1 means the operand does not have a fixed register class.
    Res += "-1, ";

  // Fill in applicable flags.
  Res += "0";

  // Ptr value whose register class is resolved via callback.
  if (OpR->isSubClassOf("PointerLikeRegClass"))
    Res += "|(1<<MCOI_LookupPtrRegClass)";

  // Predicate operands.  Check to see if the original unexpanded operand
  // was of type PredicateOp.
  if (Op.Rec->isSubClassOf("PredicateOp"))
    Res += "|(1<<MCOI_Predicate)";

  // Optional def operands.  Check to see if the original unexpanded operand
  // was of type OptionalDefOperand.
  if (Op.Rec->isSubClassOf("OptionalDefOperand"))
    Res += "|(1<<MCOI_OptionalDef)";

  // Branch target operands.  Check to see if the original unexpanded
  // operand was of type BranchTargetOperand.
  if (Op.Rec->isSubClassOf("BranchTargetOperand"))
    Res += "|(1<<MCOI_BranchTarget)";

  // Fill in operand type.
  Res += ", ";
  assert(!Op.OperandType.empty() && "Invalid operand type.");
  std::string OpTypeCpy = Op.OperandType;
  if (OpTypeCpy.find("VPRED") != std::string::npos)
    OpTypeCpy = std::regex_replace(OpTypeCpy, std::regex("OPERAND"), "OP");
  Res += OpTypeCpy.replace(OpTypeCpy.find("::"), 2, "_");

  // Fill in constraint info.
  Res += ", ";

  if (Constraint.isNone())
    Res += "0";
  else if (Constraint.isEarlyClobber())
    Res += "MCOI_EARLY_CLOBBER";
  else {
    assert(Constraint.isTied());
    Res += "MCOI_TIED_TO";
  }
}

void PrinterCapstone::instrInfoPrintDefList(
    const std::vector<Record *> &Uses, unsigned Num,
    std::string (*GetQualifiedName)(Record const *R)) const {}

void PrinterCapstone::instrInfoEmitOperandInfoTable(
    std::vector<std::string> const &OperandInfo, unsigned N) const {
  OS << "static const MCOperandInfo OperandInfo" << N << "[] = { ";
  for (const std::string &Info : OperandInfo)
    OS << "{ " << Info << " }, ";
  OS << "};\n";
}

void PrinterCapstone::instrInfoEmitMCInstrDescHdr(
    std::string TargetName) const {
  OS << "\nstatic const MCInstrDesc " << TargetName << "Insts[] = {\n";
}

void PrinterCapstone::instrInfoEmitMCInstrDescEnd() const { OS << "};\n\n"; }

void PrinterCapstone::instrInfoEmitRecord(CodeGenSchedModels const &SchedModels,
                                          CodeGenInstruction const &Inst,
                                          unsigned Num, int MinOperands) const {
  OS << "  { " << MinOperands << ", ";
}

void PrinterCapstone::instrInfoEmitTargetIndepFlags(
    CodeGenInstruction const &Inst, bool GetAllowRegisterRenaming) const {}

void PrinterCapstone::instrInfoEmitTSFFlags(uint64_t Value) const {}

void PrinterCapstone::instrInfoEmitUseDefsLists(
    std::map<std::vector<Record *>, unsigned> &EmittedLists,
    std::vector<Record *> const &UseList,
    std::vector<Record *> const &DefList) const {}

void PrinterCapstone::instrInfoEmitOperandInfo(
    std::vector<std::string> const &OperandInfo,
    OperandInfoMapTy const &OpInfo) const {
  if (OperandInfo.empty())
    OS << "0";
  else
    OS << "OperandInfo" << OpInfo.find(OperandInfo)->second;
}

void PrinterCapstone::instrInfoEmitRecordEnd(
    unsigned InstNum, std::string const &InstName) const {
  OS << " },\n";
}

void PrinterCapstone::instrInfoEmitStringLiteralDef(
    std::string const &TargetName,
    SequenceToOffsetTable<std::string> InstrNames) const {}

void PrinterCapstone::instrInfoEmitInstrNameIndices(
    std::string const &TargetName,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions,
    SequenceToOffsetTable<std::string> const &InstrNames) const {}

void PrinterCapstone::instrInfoEmitInstrDeprFeatures(
    std::string const &TargetName, std::string const &TargetNamespace,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions,
    SequenceToOffsetTable<std::string> const &InstrNames) const {}

void PrinterCapstone::instrInfoEmitInstrComplexDeprInfos(
    std::string const &TargetName,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions) const {}

void PrinterCapstone::instrInfoEmitMCInstrInfoInitRoutine(
    std::string const &TargetName, unsigned NumberedInstrSize,
    bool HasDeprecationFeatures, bool HasComplexDeprecationInfos) const {}

void PrinterCapstone::instrInfoEmitClassStruct(
    std::string const &ClassName) const {}

void PrinterCapstone::instrInfoEmitTIIHelperMethod(
    StringRef const &TargetName, Record const *Rec,
    bool ExpandDefinition) const {}

void PrinterCapstone::instrInfoEmitExternArrays(
    std::string const &TargetName, bool HasDeprecationFeatures,
    bool HasComplexDeprecationInfos) const {}

void PrinterCapstone::instrInfoEmitMCInstrInfoInit(
    std::string const &TargetName, std::string const &ClassName,
    unsigned NumberedInstrSize, bool HasDeprecationFeatures,
    bool HasComplexDeprecationInfos) const {}

void PrinterCapstone::instrInfoEmitOperandEnum(
    std::map<std::string, unsigned> const &Operands) const {}

void PrinterCapstone::instrInfoEmitGetNamedOperandIdx(
    std::map<std::string, unsigned> const &Operands,
    OpNameMapTy const &OperandMap) const {}

void PrinterCapstone::instrInfoEmitOpTypeEnumPartI() const {}

void PrinterCapstone::instrInfoEmitOpTypeEnumPartII(StringRef const &OpName,
                                                    unsigned EnumVal) const {}

void PrinterCapstone::instrInfoEmitOpTypeEnumPartIII() const {}

void PrinterCapstone::instrInfoEmitOpTypeOffsetTable(
    std::vector<int> OperandOffsets, unsigned OpRecSize,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions) const {}

void PrinterCapstone::instrInfoEmitOpcodeOpTypesTable(
    unsigned EnumVal, std::vector<Record *> const &OperandRecords,
    std::vector<int> OperandOffsets,
    ArrayRef<const CodeGenInstruction *> const &NumberedInstructions) const {}

void PrinterCapstone::instrInfoEmitGetOpTypeHdr() const {}

void PrinterCapstone::instrInfoEmitGetOpTypeReturn() const {}

void PrinterCapstone::instrInfoEmitGetOpTypeUnreachable() const {}

void PrinterCapstone::instrInfoEmitGetOpTypeEnd() const {}

void PrinterCapstone::instrInfoEmitGetMemOpSizeHdr() const {}

void PrinterCapstone::instrInfoEmitGetOpMemSizeTbl(
    std::map<int, std::vector<StringRef>> const &SizeToOperandName) const {}

std::string
PrinterCapstone::instrInfoGetInstMapEntry(StringRef const &Namespace,
                                          StringRef const &InstrName) const {
  return Namespace.str() + "_" + InstrName.str();
}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeHdr() const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeTable(
    size_t LogicalOpListSize,
    std::vector<const std::vector<unsigned> *> const &LogicalOpSizeList) const {
}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeSwitch(
    std::map<unsigned, std::vector<std::string>> InstMap) const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeReturn() const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpSizeEnd() const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpIdx() const {}

std::string
PrinterCapstone::instrInfoGetOpTypeListEntry(StringRef const &Namespace,
                                             StringRef const &OpName) const {
  return Namespace.str() + "_OpTypes_" + OpName.str();
}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeHdr() const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeTable(
    size_t OpTypeListSize,
    std::vector<const std::vector<std::string> *> const &LogicalOpTypeList)
    const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeSwitch(
    std::map<unsigned, std::vector<std::string>> InstMap) const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeReturn() const {}

void PrinterCapstone::instrInfoEmitGetLogicalOpTypeEnd() const {}

void PrinterCapstone::instrInfoEmitDeclareMCInstFeatureClasses() const {}

void PrinterCapstone::instrInfoEmitPredFcnDecl(
    RecVec const &TIIPredicates) const {}

void PrinterCapstone::instrInfoEmitPredFcnImpl(StringRef const &TargetName,
                                               RecVec const &TIIPredicates) {}

void PrinterCapstone::instrInfoEmitInstrPredVerifierIncludes() const {}

void PrinterCapstone::instrInfoEmitSubtargetFeatureBitEnumeration(
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> &SubtargetFeatures)
    const {}

void PrinterCapstone::instrInfoEmitEmitSTFNameTable(
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> &SubtargetFeatures)
    const {}

void PrinterCapstone::instrInfoEmitFeatureBitsEnum(
    std::vector<std::vector<Record *>> const &FeatureBitsets) const {}

void PrinterCapstone::instrInfoEmitFeatureBitsArray(
    std::vector<std::vector<Record *>> const &FeatureBitsets,
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> const
        &SubtargetFeatures) const {}

void PrinterCapstone::instrInfoEmitPredVerifier(
    std::vector<std::vector<Record *>> const &FeatureBitsets,
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> const
        &SubtargetFeatures,
    CodeGenTarget const &Target) const {}

void PrinterCapstone::instrInfoEmitEnums(
    CodeGenTarget const &Target, StringRef const &Namespace,
    CodeGenSchedModels const &SchedModels) const {
  emitIncludeToggle("GET_INSTRINFO_ENUM", true);

  unsigned Num = 0;
  OS << "  enum {\n";
  for (const CodeGenInstruction *Inst : Target.getInstructionsByEnumValue())
    OS << "    " << Namespace.str() << "_" << Inst->TheDef->getName()
       << "\t= " << Num++ << ",\n";
  OS << "    INSTRUCTION_LIST_END = " << Num << "\n";
  OS << "  };\n\n";
  emitIncludeToggle("GET_INSTRINFO_ENUM", false);
}

void PrinterCapstone::instrInfoEmitTIIPredicates(StringRef const &TargetName,
                                                 RecVec const &TIIPredicates,
                                                 bool ExpandDefinition) {}

void PrinterCapstone::instrInfoEmitComputeAssemblerAvailableFeatures(
    StringRef const &TargetName,
    std::map<Record *, SubtargetFeatureInfo, LessRecordByID> &SubtargetFeatures)
    const {}

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

static std::string getSecondaryOperandType(Record *Op) {
  DagInit *DAGOpInfo = Op->getValueAsDag("MIOperandInfo");
  std::string SecondaryOperandType;
  if (DAGOpInfo->getNumArgs() != 0) {
    Record *InstOpRec = cast<DefInit>(DAGOpInfo->getArg(0))->getDef();
    SecondaryOperandType = getCSOperandType(InstOpRec);
  }
  if (!SecondaryOperandType.empty())
    return " | " + SecondaryOperandType;
  return " | CS_OP_IMM";
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
    if (OperandType == "CS_OP_MEM") {
      OperandType += getSecondaryOperandType(Rec);
    }

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
  InsnOpMap << "{{ /* " + TargetName + "_" + Inst->TheDef->getName() + " - " +
                   TargetName + "_INS_" + MI->Mnemonic.upper() + " - " +
                   Inst->AsmString + " */\n";
  for (OpData const &OD : InsOps) {
    InsnOpMap.indent(2) << "{ " + OD.OpType + ", " + getCSAccess(OD.Access) +
                               " }, /* " + OD.OpAsm + " */\n";
  }
  InsnOpMap.indent(2) << "{ 0 }\n";
  InsnOpMap << "}},\n";
}

void PrinterCapstone::printInsnNameMapEnumEntry(
    StringRef const &TargetName, std::unique_ptr<MatchableInfo> const &MI,
    raw_string_ostream &InsnNameMap, raw_string_ostream &InsnEnum) const {
  static std::set<StringRef> MnemonicsSeen;
  StringRef Mnemonic = MI->Mnemonic;
  if (MnemonicsSeen.find(Mnemonic) != MnemonicsSeen.end())
    return;
  MnemonicsSeen.emplace(Mnemonic);

  std::string EnumName = TargetName.str() + "_INS_" + Mnemonic.upper();
  InsnNameMap.indent(2) << "\"" + Mnemonic + "\", // " + EnumName + "\n";
  InsnEnum.indent(2) << EnumName + ",\n";
}

void PrinterCapstone::printFeatureEnumEntry(
    StringRef const &TargetName, std::unique_ptr<MatchableInfo> const &MI,
    raw_string_ostream &FeatureEnum,
    raw_string_ostream &FeatureNameArray) const {
  static std::set<std::string> Features;
  std::string EnumName;

  for (SubtargetFeatureInfo const *STF : MI->RequiredFeatures) {
    std::string Feature = STF->TheDef->getName().str();
    if (Features.find(Feature) != Features.end())
      continue;
    Features.emplace(Feature);

    // Enum
    EnumName = TargetName.str() + "_FEATURE_" + STF->TheDef->getName().str();
    FeatureEnum << EnumName;
    if (Features.size() == 1)
      FeatureEnum << " = 128";
    FeatureEnum << ",\n";

    // Enum name map
    FeatureNameArray << "{ " + EnumName + ", \"" +
                            STF->TheDef->getName().str() + "\" },\n";
  }
}

/// Emits enum entries for each operand group.
/// The operand group name is equal printer method of the operand.
/// printSORegRegOperand -> SORegReg
void PrinterCapstone::printOpPrintGroupEnum(
    StringRef const &TargetName, std::unique_ptr<MatchableInfo> const &MI,
    raw_string_ostream &OpGroupEnum) const {
  static const std::string Exceptions[] = {
      // ARM Operand groups which are used, but are not passed here.
      "RegImmShift", "LdStmModeOperand", "MandatoryInvertedPredicateOperand"};
  static std::set<std::string> OpGroups;
  if (OpGroups.empty()) {
    for (auto OpGroup : Exceptions) {
      OpGroupEnum.indent(2) << TargetName + "_OP_GROUP_" + OpGroup + " = "
                            << OpGroups.size() << ",\n";
      OpGroups.emplace(OpGroup);
    }
  }

  CodeGenInstruction const *Inst = MI->getResultInst();
  for (const CGIOperandList::OperandInfo &Op : Inst->Operands) {
    std::string OpGroup = resolveTemplateCall(Op.PrinterMethodName).substr(5);
    if (OpGroups.find(OpGroup) != OpGroups.end())
      continue;
    OpGroupEnum.indent(2) << TargetName + "_OP_GROUP_" + OpGroup + " = "
                          << OpGroups.size() << ",\n";
    OpGroups.emplace(OpGroup);
  }
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
                                               AsmMatcherInfo &Info,
                                               StringToOffsetTable &StringTable,
                                               unsigned VariantCount) const {
  std::string InsnMapStr;
  std::string InsnOpMapStr;
  std::string InsnNameMapStr;
  std::string InsnEnumStr;
  std::string FeatureEnumStr;
  std::string FeatureNameArrayStr;
  std::string OpGroupStr;
  raw_string_ostream InsnMap(InsnMapStr);
  raw_string_ostream InsnOpMap(InsnOpMapStr);
  raw_string_ostream InsnNameMap(InsnNameMapStr);
  raw_string_ostream InsnEnum(InsnEnumStr);
  raw_string_ostream FeatureEnum(FeatureEnumStr);
  raw_string_ostream FeatureNameArray(FeatureNameArrayStr);
  raw_string_ostream OpGroups(OpGroupStr);
  addHeader(InsnMap);
  addHeader(InsnOpMap);
  addHeader(InsnNameMap);
  addHeader(InsnEnum);
  addHeader(FeatureEnum);
  addHeader(FeatureNameArray);
  addHeader(OpGroups);

  // Currently we ignore any other Asm variant then the primary.
  Record *AsmVariant = Target.getAsmParserVariant(0);
  int AsmVariantNo = AsmVariant->getValueAsInt("Variant");

  /// All our genertated tables and enums for CS mapping must have the same
  /// order as the InstrInfo instruction enum. This class sorts them after it.
  struct EnumSortComparator {
    // CGI name to position in InstrInfo Instruction enum
    std::map<const StringRef, int> InstMap;

    EnumSortComparator(const CodeGenTarget &Target) {
      ArrayRef<const CodeGenInstruction *> EnumOrdered =
          Target.getInstructionsByEnumValue();
      for (size_t I = 0; I < EnumOrdered.size(); ++I) {
        const CodeGenInstruction *CGI = EnumOrdered[I];
        InstMap[CGI->TheDef->getName()] = I;
      }
    }

    bool operator()(const std::unique_ptr<MatchableInfo> &MI1,
                    const std::unique_ptr<MatchableInfo> &MI2) const {
      const CodeGenInstruction *CGI1 = MI1->getResultInst();
      const CodeGenInstruction *CGI2 = MI2->getResultInst();
      return InstMap.at(CGI1->TheDef->getName()) <
             InstMap.at(CGI2->TheDef->getName());
    }
  };

  // Sort the Mathables like the instructions for enums.
  EnumSortComparator InstEnum(Target);
  sort(Info.Matchables, InstEnum);

  std::set<StringRef> InstSeen;
  for (const auto &MI : Info.Matchables) {
    if (MI->AsmVariantID != AsmVariantNo)
      continue;
    printInsnNameMapEnumEntry(Target.getName(), MI, InsnNameMap, InsnEnum);
    printFeatureEnumEntry(Target.getName(), MI, FeatureEnum, FeatureNameArray);
    printOpPrintGroupEnum(Target.getName(), MI, OpGroups);

    if ((MI->TheDef->getName().find("anonymous") != std::string::npos) &&
        MI->Mnemonic != "yield")
      // Hint instructions have the Mnemonic yield.
      continue;
    if (find(InstSeen, MI->TheDef->getName()) != InstSeen.end())
      continue;
    InstSeen.emplace(MI->TheDef->getName());
    printInsnMapEntry(Target.getName(), MI, InsnMap);

    printInsnOpMapEntry(Target, MI, InsnOpMap);
  }

  std::string TName = Target.getName().str();
  std::string InsnMapFilename = TName + "GenCSMappingInsn.inc";
  writeFile(InsnMapFilename, InsnMapStr);
  InsnMapFilename = TName + "GenCSMappingInsnOp.inc";
  writeFile(InsnMapFilename, InsnOpMapStr);
  InsnMapFilename = TName + "GenCSMappingInsnName.inc";
  writeFile(InsnMapFilename, InsnNameMapStr);
  InsnMapFilename = TName + "GenCSInsnEnum.inc";
  writeFile(InsnMapFilename, InsnEnumStr);
  InsnMapFilename = TName + "GenCSFeatureEnum.inc";
  writeFile(InsnMapFilename, FeatureEnumStr);
  InsnMapFilename = TName + "GenCSFeatureName.inc";
  writeFile(InsnMapFilename, FeatureNameArrayStr);
  InsnMapFilename = TName + "GenCSOpGroup.inc";
  writeFile(InsnMapFilename, OpGroupStr);
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
