import open3d as o3d
import numpy as np

def pcd_to_ply(pcd_file, ply_file):
    """
    将PCD文件转换为PLY文件
    :param pcd_file: 输入的PCD文件路径
    :param ply_file: 输出的PLY文件路径
    """
    pcd = o3d.io.read_point_cloud(pcd_file)
    o3d.io.write_point_cloud(ply_file, pcd, write_ascii=True)
    print(f"Converted {pcd_file} to {ply_file}")

def ply_to_pcd(ply_file, pcd_file):
    """
    将PLY文件转换为PCD文件
    :param ply_file: 输入的PLY文件路径
    :param pcd_file: 输出的PCD文件路径
    """
    ply = o3d.io.read_point_cloud(ply_file)
    o3d.io.write_point_cloud(pcd_file, ply, write_ascii=True)
    print(f"Converted {ply_file} to {pcd_file}")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert between PCD and PLY formats")
    parser.add_argument("input_file", type=str, help="Input file path (PCD or PLY)", default="/home/gc/workplace/Sentry/map_build/fast-lio/src/FAST_LIO/PCD/scans.pcd")
    parser.add_argument("output_file", type=str, help="Output file path (PLY or PCD)", default="/home/gc/workplace/Sentry/map_build/fast-lio/src/FAST_LIO/ply")
    args = parser.parse_args()

    input_ext = args.input_file.split('.')[-1].lower()
    output_ext = args.output_file.split('.')[-1].lower()

    if input_ext == "pcd" and output_ext == "ply":
        pcd_to_ply(args.input_file, args.output_file)
    elif input_ext == "ply" and output_ext == "pcd":
        ply_to_pcd(args.input_file, args.output_file)
    else:
        print("Unsupported file format conversion. Please use PCD to PLY or PLY to PCD.")