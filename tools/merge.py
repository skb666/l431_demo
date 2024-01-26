#!/usr/bin/python
import sys
import json

if __name__ == "__main__":
    partition_table  = "partition_table.json"
    bld_file = "../bld/emStudio/Output/Debug/Exe/l431_bld.bin"
    app_file = "../app/emStudio/Output/Debug/Exe/l431_app.bin"
    # bld_file = "../bld/build/l431_bld.bin"
    # app_file = "../app/build/l431_app.bin"
    out_file = "merge.bin"

    with open(partition_table, "r") as f_obj:
        flash = json.load(f_obj)
    
    bld_bin = b''
    app_bin = b''
    for part in flash:
        if part["name"] == "bld":
            with open(bld_file, "rb") as f_obj:
                bld_bin = f_obj.read()
                bld_bin += b'\xff' * (int(part["size"], 16) - len(bld_bin))
        elif part["name"] == "app":
            with open(app_file, "rb") as f_obj:
                app_bin = f_obj.read()
            break
        else:
            bld_bin += b'\xff' * int(part["size"], 16)

    with open(out_file, 'wb') as f_obj:
        f_obj.write(bld_bin + app_bin)
