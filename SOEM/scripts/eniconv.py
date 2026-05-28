#!/usr/bin/env python3

import argparse
from xml.etree import ElementTree
from sys import stdin, stdout

## Simple C generator

# CGen 用于按 C 语言初始化器格式输出代码，同时维护缩进、逗号和分号。
class CGen:
   def __init__(self, file, columns = 3, /):
      self.file = file
      self.tab = " " * columns
      self.count = 0
      self.content = False
   def newline(self):
      # 根据上一行是否已有内容，决定输出逗号、分号或空行。
      if self.content:
         if self.count > 0:
            self.file.write(",\n")
         else:
            self.file.write(";\n\n")
         self.content = False
      else:
         self.file.write("\n")
   def prep(self, line):
      # 输出 C 预处理指令，例如 include。
      if self.content:
         self.newline()
      self.file.write(f"#{line}\n")
   def line(self, line):
      # 输出一行 C 初始化字段或普通语句，并记录当前行是否有内容。
      self.newline()
      self.file.write(self.tab * self.count)
      self.file.write(line)
      self.content = not not line
   def open(self, line = "", c = 1, /):
      # 打开一层或多层 C 初始化大括号，可选地生成变量定义。
      self.newline()
      self.file.write(self.tab * self.count)
      if line:
         self.file.write(f"{line} = ")
      self.file.write("{" * c)
      self.count += c
      self.content = False
   def close(self, c = 1, /):
      # 关闭一层或多层 C 初始化大括号。
      self.count -= c
      self.file.write(f"\n{self.tab * self.count}{'}' * c}")
      self.content = True
   def comment(self, lines):
      # 将 ENI 中的 Comment 字段转换为 C++ 风格注释。
      for line in [l.rstrip('\\') for l in lines.strip().split("\n")]:
         self.newline()
         self.file.write(f"{self.tab * self.count}// {line}")

## XML parsing convenience

# 获取指定路径下的 XML 子元素，并可校验数量范围。
def getElements(element, path, min = None, max = None, /):
   elements = element.findall(f"./{path}")
   if (min is not None):
      l = len(elements)
      if max is not None:
         if max < l:
            raise(Exception(f"Too many {path} elements."))
      if l < min:
         raise(Exception(f"Too few {path} elements."))
   return elements

# 获取且只允许获取一个 XML 子元素。
def getElement(element, path):
   return getElements(element, path, 1, 1)[0]

# 读取整数文本，base=0 支持十进制和 0x 前缀十六进制。
def getInt(element, path):
   return int(getElement(element, path).text, base = 0)

# 读取可选整数，不存在时返回默认值。
def getOptInt(element, path, default = None, /):
   es = getElements(element, path, 0, 1)
   if es: return int(es[0].text, base = 0)
   return default

# 读取必选文本。
def getText(element, path):
   return getElement(element, path).text

# 读取可选文本，不存在时返回默认值。
def getOptText(element, path, default = "", /):
   es = getElements(element, path, 0, 1)
   if es: return es[0].text
   return default

## ENI parsing

# 解析 ENI 中单条 CoE InitCmd，并转换为后续 C 结构体所需字段。
def parseCoEInitCmd(element):
   if (element.get("CompleteAccess", "0") == "1"):
      ca = "TRUE"
   else:
      ca = "FALSE"
   return {
         "Comment": getOptText(element, "Comment"),
         "Transition": [t.text for t in getElements(element, "Transition")],
         "CA": ca,
         "Ccs": (0xff & getInt(element, "Ccs")),
         "Index": (0xffff & getInt(element, "Index")),
         "SubIdx": (0xff & getInt(element, "SubIndex")),
         "Timeout": (1000 * getInt(element, "Timeout")),
         "Data": list(bytearray.fromhex(getOptText(element, "Data")))
         }

# 解析 CoE InitCmd 列表，跳过 Disabled=1 的初始化命令。
def parseCoEInitCmds(elements):
   return [parseCoEInitCmd(e) for e in elements if getOptText(e, "Disabled") != "1"]

# 解析单个从站的地址、身份信息和 CoE 初始化命令。
def parseSlave(element, slave):
   return {
         "Slave": (0xffff & (1 - getOptInt(element, "Info/AutoIncAddr", (1 - slave)))),
         "VendorId": (0xffffffff & getInt(element, "Info/VendorId")),
         "ProductCode": (0xffffffff & getInt(element, "Info/ProductCode")),
         "RevisionNo": (0xffffffff & getInt(element, "Info/RevisionNo")),
         "CoECmds": parseCoEInitCmds(getElements(element, "Mailbox/CoE/InitCmds/InitCmd"))
         }

# 解析 ENI 配置中的全部从站。
def parseSlaves(element):
   es = getElements(element, "Slave")
   return [parseSlave(es[s], (s + 1)) for s in range(len(es))]

# 解析 ENI 的 Config 节点，目前只提取从站相关配置。
def parseConfig(element):
   return {"slave": parseSlaves(element)}

## Output generation

# 将 ENI 中的状态转换名转换为 SOEM 的 ECT_ESMTRANS_* 位掩码表达式。
def genTransition(transitions):
   tt = ["ECT_ESMTRANS_" + t for t in transitions]
   if len(tt) > 1:
      return f"({' | '.join(tt)})"
   return tt[0]

# 为某个从站生成所有 CoE 初始化命令使用的数据字节数组，并返回每条命令的数据指针。
def genCoEData(cg, slave, cmds):
   cName = f"s{slave}_coeData"
   size = sum(len(c["Data"]) for c in cmds)
   if size > 0:
      cg.open(f"static uint8 {cName}[{size}]")
      for data in [c["Data"] for c in cmds if len(c["Data"]) > 0]:
         cg.line(", ".join([f"{b}" for b in data]))
      cg.close()
   size = 0
   coeData = list()
   for s in [len(c["Data"]) for c in cmds]:
      if s > 0:
         offset = size
         size += s
         coeData.append((s, f"({cName} + {offset})"))
      else:
         coeData.append((0, "NULL"))
   return coeData

# 生成某个从站的 ec_enicoecmdt 数组。
def genCoECmds(cg, slave, cmds):
   noCoeCmds = len(cmds)
   if noCoeCmds == 0:
      cName = "NULL"
   else:
      cName = f"s{slave}_coeCmds"
      coeData = genCoEData(cg, slave, cmds)
      cg.open(f"static ec_enicoecmdt {cName}[{noCoeCmds}]")
      for c, (DataSize, Data) in zip(cmds, coeData):
         cg.open()
         if c["Comment"]:
            cg.comment(c["Comment"])
         cg.line(f".Transition = {genTransition(c['Transition'])}")
         for f in ["CA", "Ccs"]:
            cg.line(f".{f} = {c[f]}")
         cg.line(".Index = 0x{:04X}".format(c["Index"]))
         for f in ["SubIdx", "Timeout"]:
            cg.line(f".{f} = {c[f]}")
         cg.line(f".DataSize = {DataSize}")
         cg.line(f".Data = {Data}")
         cg.close()
      cg.close()
   return (noCoeCmds, cName)

# 生成 ec_enislavet 从站数组，只包含存在 CoE 初始化命令的从站。
def genSlaves(cg, slaves):
   coeList = [genCoECmds(cg, s["Slave"], s["CoECmds"]) for s in slaves]
   noSlaves = sum(n > 0 for n, _ in coeList)
   if noSlaves == 0:
      cName = "NULL"
   else:
      cName = "eniSlave"
      cg.open(f"static ec_enislavet {cName}[{noSlaves}]")
      for s, (CoECmdCount, CoECmds) in zip(slaves, coeList):
         if CoECmdCount > 0:
            cg.open()
            for f in ["Slave", "VendorId", "ProductCode", "RevisionNo"]:
               cg.line(f".{f} = {s[f]}")
            cg.line(f".CoECmds = {CoECmds}")
            cg.line(f".CoECmdCount = {CoECmdCount}")
            cg.close()
      cg.close()
   return (noSlaves, cName)

# 生成最终导出的 ec_eni 配置对象。
def genConfig(cg, config):
   slaves = sorted(config["slave"], key = lambda s: s["Slave"])
   slavecount, slave = genSlaves(cg, slaves)
   cg.open("ec_enit ec_eni")
   cg.line(f".slave = {slave}")
   cg.line(f".slavecount = {slavecount}")
   cg.close()

# 生成完整 C 文件内容。
def genFile(file, config):
   cg = CGen(file)
   cg.prep("include \"soem/soem.h\"")
   genConfig(cg, config)
   cg.newline()

## Interface

# 读取 ENI XML 文件，兼容根节点为 EtherCATConfig 或外层再包一层的格式。
def parseENI(file):
   tree = ElementTree.parse(file).getroot()
   if tree.tag == "EtherCATConfig":
      path = "Config"
   else:
      path = "EtherCATConfig/Config"
   return parseConfig(getElement(tree, path))

# 主处理流程：解析 ENI，然后写出 C 文件。
def process(args):
   genFile(args.outfile, parseENI(args.eni))

## Program execution

# 解析命令行参数：输入 ENI 文件，输出 C 文件可选，默认输出到 stdout。
def parseArgs():
   parser = argparse.ArgumentParser(
         description = "Convert an ENI file to a C file suited for an SOEM application.")
   parser.add_argument("eni", help = "the ENI file to convert",
         type = argparse.FileType("r"))
   parser.add_argument("outfile", help = "the output C file", nargs="?",
         type = argparse.FileType("w"), default = stdout)
   return parser.parse_args()

# 捕获转换过程中的异常，并以简洁错误信息退出。
try:
   process(parseArgs())
except Exception as e:
   raise SystemExit(f"Failure: {e}.") from None

