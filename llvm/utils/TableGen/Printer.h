//===--------------- Printer.h - Printer Interface --------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_UTILS_TABLEGEN_PRINTER_H
#define LLVM_UTILS_TABLEGEN_PRINTER_H

#include "CodeGenRegisters.h"
#include "CodeGenTarget.h"
#include "PrinterTypes.h"
#include "RegisterInfoEmitterTypes.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {

class PrinterBitVectorEmitter {
  BitVector Values;

public:
  virtual void add(unsigned V);
  virtual void print(raw_ostream &OS);
};

void printBitVectorAsHex(raw_ostream &OS, const BitVector &Bits,
                         unsigned Width);

//==============================
//
// Implementation: LLVM
//
//==============================

/// Interface for printing the generated code.
/// Every string which will be in the generated code of a backend originates
/// from here.
///
/// It also is the only class which writes directly into the output stream for
/// the backend.
///
/// This class has methods for all classes of backends which emit generated
/// code. If a backend currently does not emit the code in a language you need
/// you simply inherit this class and implement the relevant methods.
///
/// Printer implementation of LLVM.
/// This is the default printer for all backends.
///
/// Output language: C++
class PrinterLLVM {

private:
  formatted_raw_ostream &OS;

public:
  PrinterLLVM(formatted_raw_ostream &OS);

  virtual ~PrinterLLVM();

  static PrinterLanguage getLanguage();

  virtual void flushOS() const { OS.flush(); }

  //--------------------------
  // General printing methods
  //--------------------------

  virtual void emitNewline(unsigned Count) const {
    for (unsigned I = Count; I > 0; --I)
      OS << "\n";
  }
  virtual void emitString(std::string const &Str) const { OS << Str; }

  //--------------------------
  // General printing methods
  //--------------------------

  virtual void emitNamespace(std::string const &Name, bool Begin,
                             std::string const &Comment = "") const;
  virtual void emitIncludeToggle(std::string const &Name, bool Begin) const;

  //------------------------
  // Backend: RegisterInfo
  //------------------------

  virtual void regInfoEmitSourceFileHeader(std::string const &Desc) const;
  virtual void regInfoEmitEnums(CodeGenTarget const &Target,
                                CodeGenRegBank const &Bank) const;
  virtual void
  regInfoEmitRegDiffLists(std::string const TargetName,
                          SequenceToOffsetTable<DiffVec> const &DiffSeqs) const;
  virtual void regInfoEmitLaneMaskLists(
      std::string const TargetName,
      SequenceToOffsetTable<MaskVec> const &DiffSeqs) const;
  virtual void regInfoEmitSubRegIdxLists(
      std::string const TargetName,
      SequenceToOffsetTable<SubRegIdxVec, deref<std::less<>>> const
          &SubRegIdxSeqs) const;
  virtual void regInfoEmitSubRegIdxSizes(
      std::string const TargetName,
      std::deque<CodeGenSubRegIndex> const &SubRegIndices) const;
  virtual void regInfoEmitSubRegStrTable(
      std::string const TargetName,
      SequenceToOffsetTable<std::string> const &RegStrings) const;
  virtual void regInfoEmitRegDesc(
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
      SequenceToOffsetTable<std::string> const &RegStrings) const;
  virtual void regInfoEmitRegUnitRoots(std::string const TargetName,
                                       CodeGenRegBank const &RegBank) const;
  virtual void
  regInfoEmitRegClasses(std::list<CodeGenRegisterClass> const &RegClasses,
                        SequenceToOffsetTable<std::string> &RegClassStrings,
                        CodeGenTarget const &Target) const;
  virtual void regInfoEmitStrLiteralRegClasses(
      std::string const TargetName,
      SequenceToOffsetTable<std::string> const &RegClassStrings) const;
  virtual void regInfoEmitMCRegClassesTable(
      std::string const TargetName,
      std::list<CodeGenRegisterClass> const &RegClasses,
      SequenceToOffsetTable<std::string> &RegClassStrings) const;
  virtual void
  regInfoEmitRegEncodingTable(std::string const TargetName,
                              std::deque<CodeGenRegister> const &Regs) const;
  virtual void regInfoEmitMCRegInfoInit(
      std::string const TargetName, CodeGenRegBank const &RegBank,
      std::deque<CodeGenRegister> const &Regs,
      std::list<CodeGenRegisterClass> const &RegClasses,
      std::deque<CodeGenSubRegIndex> const &SubRegIndices) const;
  virtual void regInfoEmitInfoDwarfRegs(StringRef const &Namespace,
                                        DwarfRegNumsVecTy &DwarfRegNums,
                                        unsigned MaxLength, bool IsCtor) const;
  virtual void regInfoEmitInfoDwarfRegsRev(StringRef const &Namespace,
                                           DwarfRegNumsVecTy &DwarfRegNums,
                                           unsigned MaxLength,
                                           bool IsCtor) const;
  virtual void regInfoEmitInfoRegMapping(StringRef const &Namespace,
                                         unsigned MaxLength, bool IsCtor) const;
  virtual void regInfoEmitHeaderIncludes() const;
  virtual void regInfoEmitHeaderExternRegClasses(
      std::list<CodeGenRegisterClass> const &RegClasses) const;
  virtual void regInfoEmitHeaderDecl(std::string const &TargetName,
                                     std::string const &ClassName,
                                     bool SubRegsPresent,
                                     bool DeclareGetPhysRegBaseClass) const;
  virtual void
  regInfoEmitExternRegClassesArr(std::string const &TargetName) const;
  virtual void regInfoEmitVTSeqs(
      SequenceToOffsetTable<std::vector<MVT::SimpleValueType>> const &VTSeqs)
      const;
  virtual void regInfoEmitSubRegIdxTable(
      std::deque<CodeGenSubRegIndex> const &SubRegIndices) const;
  virtual void regInfoEmitRegClassInfoTable(
      std::list<CodeGenRegisterClass> const &RegClasses,
      SequenceToOffsetTable<std::vector<MVT::SimpleValueType>> const &VTSeqs,
      CodeGenHwModes const &CGH, unsigned NumModes) const;
  virtual void regInfoEmitSubClassMaskTable(
      std::list<CodeGenRegisterClass> const &RegClasses,
      SmallVector<IdxList, 8> &SuperRegIdxLists,
      SequenceToOffsetTable<IdxList, deref<std::less<>>> &SuperRegIdxSeqs,
      std::deque<CodeGenSubRegIndex> const &SubRegIndices,
      BitVector &MaskBV) const;
  virtual void regInfoEmitSuperRegIdxSeqsTable(
      SequenceToOffsetTable<IdxList, deref<std::less<>>> const &SuperRegIdxSeqs)
      const;
  virtual void regInfoEmitSuperClassesTable(
      std::list<CodeGenRegisterClass> const &RegClasses) const;
  virtual void
  regInfoEmitRegClassMethods(std::list<CodeGenRegisterClass> const &RegClasses,
                             std::string const &TargetName) const;
  virtual void regInfomitRegClassInstances(
      std::list<CodeGenRegisterClass> const &RegClasses,
      SequenceToOffsetTable<IdxList, deref<std::less<>>> const &SuperRegIdxSeqs,
      SmallVector<IdxList, 8> const &SuperRegIdxLists,
      std::string const &TargetName) const;
  virtual void regInfoEmitRegClassTable(
      std::list<CodeGenRegisterClass> const &RegClasses) const;
  virtual void
  regInfoEmitCostPerUseTable(std::vector<unsigned> const &AllRegCostPerUse,
                             unsigned NumRegCosts) const;
  virtual void
  regInfoEmitInAllocatableClassTable(llvm::BitVector const &InAllocClass) const;
  virtual void regInfoEmitRegExtraDesc(std::string const &TargetName,
                                       unsigned NumRegCosts) const;
  virtual void regInfoEmitSubClassSubRegGetter(
      std::string const &ClassName, unsigned SubRegIndicesSize,
      std::deque<CodeGenSubRegIndex> const &SubRegIndices,
      std::list<CodeGenRegisterClass> const &RegClasses,
      CodeGenRegBank &RegBank) const;
  virtual void regInfoEmitRegClassWeight(CodeGenRegBank const &RegBank,
                                         std::string const &ClassName) const;
  virtual void regInfoEmitRegUnitWeight(CodeGenRegBank const &RegBank,
                                        std::string const &ClassName,
                                        bool RegUnitsHaveUnitWeight) const;
  virtual void regInfoEmitGetNumRegPressureSets(std::string const &ClassName,
                                                unsigned NumSets) const;
  virtual void regInfoEmitGetRegPressureTables(CodeGenRegBank const &RegBank,
                                               std::string const &ClassName,
                                               unsigned NumSets) const;
  virtual void regInfoEmitRCSetsTable(
      std::string const &ClassName, unsigned NumRCs,
      SequenceToOffsetTable<std::vector<int>> const &PSetsSeqs,
      std::vector<std::vector<int>> const &PSets) const;
  virtual void regInfoEmitGetRegUnitPressureSets(
      SequenceToOffsetTable<std::vector<int>> const &PSetsSeqs,
      CodeGenRegBank const &RegBank, std::string const &ClassName,
      std::vector<std::vector<int>> const &PSets) const;
  virtual void regInfoEmitExternTableDecl(std::string const &TargetName) const;
  virtual void
  regInfoEmitRegClassInit(std::string const &TargetName,
                          std::string const &ClassName,
                          CodeGenRegBank const &RegBank,
                          std::list<CodeGenRegisterClass> const &RegClasses,
                          std::deque<CodeGenRegister> const &Regs,
                          unsigned SubRegIndicesSize) const;
  virtual void regInfoEmitSaveListTable(Record const *CSRSet,
                                        SetTheory::RecVec const *Regs) const;
  virtual void regInfoEmitRegMaskTable(std::string const &CSRSetName,
                                       BitVector &Covered) const;
  virtual void
  regInfoEmitIsConstantPhysReg(std::deque<CodeGenRegister> const &Regs,
                               std::string const &ClassName) const;
  virtual void regInfoEmitGetRegMasks(std::vector<Record *> const &CSRSets,
                                      std::string const &ClassName) const;
  virtual void regInfoEmitGPRCheck(
      std::string const &ClassName,
      std::list<CodeGenRegisterCategory> const &RegCategories) const;
  virtual void regInfoEmitFixedRegCheck(
      std::string const &ClassName,
      std::list<CodeGenRegisterCategory> const &RegCategories) const;
  virtual void regInfoEmitArgRegCheck(
      std::string const &ClassName,
      std::list<CodeGenRegisterCategory> const &RegCategories) const;
  virtual void regInfoEmitGetRegMaskNames(std::vector<Record *> const &CSRSets,
                                          std::string const &ClassName) const;
  virtual void regInfoEmitGetFrameLowering(std::string const &TargetName) const;
  virtual void
  regInfoEmitComposeSubRegIndicesImplHead(std::string const &ClName) const;
  virtual void regInfoEmitComposeSubRegIndicesImplBody(
      SmallVector<SmallVector<CodeGenSubRegIndex *, 4>, 4> const &Rows,
      unsigned SubRegIndicesSize, SmallVector<unsigned, 4> const &RowMap) const;
  virtual void regInfoEmitLaneMaskComposeSeq(
      SmallVector<SmallVector<MaskRolPair, 1>, 4> const &Sequences,
      SmallVector<unsigned, 4> const &SubReg2SequenceIndexMap,
      std::deque<CodeGenSubRegIndex> const &SubRegIndices) const;
  virtual void regInfoEmitComposeSubRegIdxLaneMask(
      std::string const &ClName,
      std::deque<CodeGenSubRegIndex> const &SubRegIndices) const;
  virtual void regInfoEmitComposeSubRegIdxLaneMaskRev(
      std::string const &ClName,
      std::deque<CodeGenSubRegIndex> const &SubRegIndices) const;
  virtual void regInfoEmitRegBaseClassMapping(
      std::string const &ClassName,
      SmallVector<const CodeGenRegisterClass *> const BaseClasses,
      std::vector<uint8_t> const Mapping) const;
};

#endif // LLVM_UTILS_TABLEGEN_PRINTER_H

} // end namespace llvm
