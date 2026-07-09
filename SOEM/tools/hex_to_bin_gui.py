#!/usr/bin/env python3

import argparse
import re
import tkinter as tk
from tkinter import messagebox

HEX_DIGIT_RE = re.compile(r"[0-9a-fA-F]")
HEX_PREFIX_RE = re.compile(r"0[xX]")
SEPARATOR_RE = re.compile(r"[\s,_:\-]+")


def normalize_hex_text(text):
   text = HEX_PREFIX_RE.sub("", text)
   text = SEPARATOR_RE.sub("", text)
   if not text:
      raise ValueError("请输入十六进制内容。")
   invalid = sorted(set(ch for ch in text if not HEX_DIGIT_RE.fullmatch(ch)))
   if invalid:
      raise ValueError("包含非法字符: " + " ".join(invalid))
   return text.upper()


def hex_to_binary(text, group_bits = 4, pad_left = True):
   hex_text = normalize_hex_text(text)
   bit_count = len(hex_text) * 4
   value = int(hex_text, 16)
   binary = format(value, f"0{bit_count}b") if pad_left else format(value, "b")
   if group_bits and group_bits > 0:
      binary = " ".join(binary[i:i + group_bits] for i in range(0, len(binary), group_bits))
   return binary


def copy_to_clipboard(root, text):
   root.clipboard_clear()
   root.clipboard_append(text)
   root.update()


def create_gui():
   root = tk.Tk()
   root.title("十六进制转二进制工具")
   root.geometry("760x520")
   root.minsize(620, 420)

   pad_left_var = tk.BooleanVar(value = True)
   group_bits_var = tk.StringVar(value = "4")
   status_var = tk.StringVar(value = "支持 0x/0X 前缀、大小写字母、空格、逗号、下划线、冒号、短横线分隔。")

   root.columnconfigure(0, weight = 1)
   root.rowconfigure(1, weight = 1)
   root.rowconfigure(4, weight = 1)

   title = tk.Label(root, text = "十六进制转二进制", font = ("Sans", 16, "bold"))
   title.grid(row = 0, column = 0, sticky = "w", padx = 14, pady = (12, 6))

   input_frame = tk.LabelFrame(root, text = "十六进制输入")
   input_frame.grid(row = 1, column = 0, sticky = "nsew", padx = 14, pady = 6)
   input_frame.rowconfigure(0, weight = 1)
   input_frame.columnconfigure(0, weight = 1)

   input_text = tk.Text(input_frame, height = 7, wrap = "word", undo = True)
   input_text.grid(row = 0, column = 0, sticky = "nsew", padx = 8, pady = 8)
   input_text.insert("1.0", "0x1A 2b, FF")

   options_frame = tk.Frame(root)
   options_frame.grid(row = 2, column = 0, sticky = "ew", padx = 14, pady = 4)
   options_frame.columnconfigure(5, weight = 1)

   tk.Checkbutton(options_frame, text = "按原始位宽补前导 0", variable = pad_left_var).grid(row = 0, column = 0, sticky = "w")
   tk.Label(options_frame, text = "分组位数:").grid(row = 0, column = 1, padx = (16, 4), sticky = "w")
   group_entry = tk.Entry(options_frame, textvariable = group_bits_var, width = 6)
   group_entry.grid(row = 0, column = 2, sticky = "w")
   tk.Label(options_frame, text = "填 0 表示不分组").grid(row = 0, column = 3, padx = (8, 0), sticky = "w")

   button_frame = tk.Frame(root)
   button_frame.grid(row = 3, column = 0, sticky = "ew", padx = 14, pady = 6)

   output_frame = tk.LabelFrame(root, text = "二进制输出")
   output_frame.grid(row = 4, column = 0, sticky = "nsew", padx = 14, pady = 6)
   output_frame.rowconfigure(0, weight = 1)
   output_frame.columnconfigure(0, weight = 1)

   output_text = tk.Text(output_frame, height = 8, wrap = "word")
   output_text.grid(row = 0, column = 0, sticky = "nsew", padx = 8, pady = 8)

   def convert():
      try:
         group_bits = int(group_bits_var.get())
         if group_bits < 0:
            raise ValueError("分组位数不能小于 0。")
         result = hex_to_binary(input_text.get("1.0", "end"), group_bits, pad_left_var.get())
         output_text.delete("1.0", "end")
         output_text.insert("1.0", result)
         status_var.set("转换成功。")
      except ValueError as err:
         status_var.set(str(err))
         messagebox.showerror("输入错误", str(err), parent = root)

   def clear_all():
      input_text.delete("1.0", "end")
      output_text.delete("1.0", "end")
      status_var.set("已清空。")

   def copy_result():
      result = output_text.get("1.0", "end").strip()
      if result:
         copy_to_clipboard(root, result)
         status_var.set("结果已复制到剪贴板。")
      else:
         status_var.set("没有可复制的结果。")

   tk.Button(button_frame, text = "转换", command = convert, width = 12).pack(side = "left")
   tk.Button(button_frame, text = "复制结果", command = copy_result, width = 12).pack(side = "left", padx = 8)
   tk.Button(button_frame, text = "清空", command = clear_all, width = 12).pack(side = "left")

   status = tk.Label(root, textvariable = status_var, anchor = "w", fg = "#444")
   status.grid(row = 5, column = 0, sticky = "ew", padx = 14, pady = (4, 10))

   root.bind("<Control-Return>", lambda event: convert())
   convert()
   root.mainloop()


def parse_args():
   parser = argparse.ArgumentParser(description = "Convert hexadecimal text to binary.")
   parser.add_argument("hex", nargs = "*", help = "hexadecimal input, for example: 0x1A FF")
   parser.add_argument("-g", "--group-bits", type = int, default = 4, help = "group output every N bits, use 0 to disable grouping")
   parser.add_argument("--no-pad", action = "store_true", help = "do not keep leading zero bits from the original hex width")
   parser.add_argument("--gui", action = "store_true", help = "start GUI mode")
   return parser.parse_args()


def main():
   args = parse_args()
   if args.gui or not args.hex:
      create_gui()
      return
   try:
      print(hex_to_binary(" ".join(args.hex), args.group_bits, not args.no_pad))
   except ValueError as err:
      raise SystemExit(f"Failure: {err}") from None


if __name__ == "__main__":
   main()
