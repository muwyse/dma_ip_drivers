
import sys
import math

if __name__ == "__main__":

  if len(sys.argv) == 3:
    filename = sys.argv[1]
    size_mb = int(sys.argv[2])
    print(size_mb)

    dwords = size_mb * 1024 * 1024 / 8
    print(dwords)

    # read nbf file
    f = open(filename, "w")

    addr = 0x80000000
    for i in range(dwords):
      s = "03_" + "{0:016x}".format(addr) + "_{0:016x}".format(0) + "\n"
      f.write(s)
      addr += 8

    f.write("fe_{0:016x}_{0:016x}\n".format(0))
    f.write("ff_{0:016x}_{0:016x}\n".format(0))

    f.close()

  else:
    print("USAGE:")
    command = "python zero_gen.py filename.nbf size (in MiB)"
    print(command)
