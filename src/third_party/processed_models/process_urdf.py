#!/usr/bin/env python3
"""
通用URDF文件处理工具

功能：
- 处理URDF文件中的xacro语法
- 替换package://路径为相对路径
- 移除xacro变量引用
- 复制相关的网格文件
- 支持批量处理多个URDF文件

使用方法：
- 基本用法：python3 process_urdf.py --input <urdf_dir> --output <output_dir>
- 自定义package路径：python3 process_urdf.py --input <urdf_dir> --output <output_dir> --package <package_name>:<relative_path>
- 显示帮助：python3 process_urdf.py --help
"""
import os
import re
import argparse
import subprocess
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
def process_urdf_file(input_file, output_file, package_mappings):
    """处理单个URDF文件
    
    Args:
        input_file (str): 输入URDF文件路径
        output_file (str): 输出URDF文件路径
        package_mappings (dict): package路径映射字典
    """
    try:
        # 读取文件内容
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 替换xacro语法
        # 1. 替换$(arg ...)为空白
        content = re.sub(r'\$\(arg\s+[^)]+\)', '', content)
        # 2. 替换${...}为空白
        content = re.sub(r'\$\{[^}]+\}', '', content)
        # 3. 替换package://路径
        for package_name, relative_path in package_mappings.items():
            pattern = rf'package://{re.escape(package_name)}/'
            content = re.sub(pattern, relative_path, content)
        
        # 写入处理后的文件
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        
        logging.info(f"处理完成: {input_file} -> {output_file}")
        return True
    except Exception as e:
        logging.error(f"处理文件失败 {input_file}: {str(e)}")
        return False

def copy_meshes(input_dir, output_dir):
    """复制meshes目录
    
    Args:
        input_dir (str): 输入目录
        output_dir (str): 输出目录
    """
    try:
        # 查找meshes目录
        meshes_dirs = []
        for root, dirs, files in os.walk(input_dir):
            if 'meshes' in dirs:
                meshes_dir = os.path.join(root, 'meshes')
                meshes_dirs.append(meshes_dir)
                logging.info(f"找到meshes目录: {meshes_dir}")
        
        # 复制每个meshes目录
        for meshes_dir in meshes_dirs:
            # 计算相对路径，保持目录结构
            rel_path = os.path.relpath(meshes_dir, input_dir)
            output_meshes_dir = os.path.join(output_dir, rel_path)
            
            # 使用rsync复制目录
            subprocess.run(
                ["rsync", "-av", "--progress", meshes_dir, output_dir],
                check=True,
                capture_output=True,
                text=True
            )
            logging.info(f"复制完成: {meshes_dir} -> {output_meshes_dir}")
        
        return True
    except Exception as e:
        logging.error(f"复制meshes目录失败: {str(e)}")
        return False

def process_directory(input_dir, output_dir, package_mappings):
    """处理目录中的所有URDF文件
    
    Args:
        input_dir (str): 输入目录
        output_dir (str): 输出目录
        package_mappings (dict): package路径映射字典
    """
    processed_count = 0
    failed_count = 0
    
    # 遍历目录中的所有文件
    for root, dirs, files in os.walk(input_dir):
        for file_name in files:
            if file_name.endswith('.urdf'):
                input_file = os.path.join(root, file_name)
                # 计算相对路径，保持目录结构
                rel_path = os.path.relpath(input_file, input_dir)
                output_file = os.path.join(output_dir, rel_path)
                
                # 处理文件
                if process_urdf_file(input_file, output_file, package_mappings):
                    processed_count += 1
                else:
                    failed_count += 1
    
    logging.info(f"目录处理完成: {processed_count} 成功, {failed_count} 失败")
    return processed_count > 0

def parse_package_mappings(package_args):
    """解析package映射参数
    
    Args:
        package_args (list): package映射参数列表
    
    Returns:
        dict: package路径映射字典
    """
    mappings = {}
    
    for arg in package_args:
        if ':' in arg:
            package_name, relative_path = arg.split(':', 1)
            mappings[package_name] = relative_path
        else:
            # 默认映射：package_name -> ../
            mappings[arg] = '../'
    
    return mappings

def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='通用URDF文件处理工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例：

1. 处理单个文件：
   python3 process_urdf.py --input robot.urdf --output processed_robot.urdf

2. 处理目录：
   python3 process_urdf.py --input urdf_dir --output processed_dir

3. 自定义package路径：
   python3 process_urdf.py --input urdf_dir --output processed_dir \
       --package my_robot_description:../meshes

4. 多个package映射：
   python3 process_urdf.py --input urdf_dir --output processed_dir \
       --package my_robot_description:../meshes \
       --package common_description:../../common/meshes
        """
    )
    
    parser.add_argument('--input', '-i', required=True, 
                        help='输入URDF文件或目录路径')
    parser.add_argument('--output', '-o', required=True, 
                        help='输出文件或目录路径')
    parser.add_argument('--package', '-p', action='append', default=[],
                        help='package路径映射，格式: package_name:relative_path')
    parser.add_argument('--copy-meshes', '-c', action='store_true', default=True,
                        help='复制meshes目录到输出目录')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='显示详细日志')
    
    args = parser.parse_args()
    
    # 设置日志级别
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # 解析package映射
    package_mappings = parse_package_mappings(args.package)
    
    # 如果没有指定package映射，添加默认映射
    if not package_mappings:
        # 默认映射常见的package名称
        package_mappings = {
            'turtlebot3_description': '../',
            'robot_description': '../',
            'description': '../'
        }
        logging.info('使用默认package映射: %s', package_mappings)
    else:
        logging.info('使用自定义package映射: %s', package_mappings)
    
    # 处理输入
    if os.path.isfile(args.input):
        # 处理单个文件
        if not args.input.endswith('.urdf'):
            logging.error('输入文件必须是.urdf文件')
            return 1
        
        # 确保输出目录存在
        os.makedirs(os.path.dirname(args.output), exist_ok=True)
        
        # 处理文件
        success = process_urdf_file(args.input, args.output, package_mappings)
        
        # 复制meshes目录（如果需要）
        if success and args.copy_meshes:
            input_dir = os.path.dirname(args.input)
            output_dir = os.path.dirname(args.output)
            copy_meshes(input_dir, output_dir)
    
    elif os.path.isdir(args.input):
        # 处理目录
        os.makedirs(args.output, exist_ok=True)
        success = process_directory(args.input, args.output, package_mappings)
        
        # 复制meshes目录（如果需要）
        if success and args.copy_meshes:
            copy_meshes(args.input, args.output)
    
    else:
        logging.error('输入路径不存在: %s', args.input)
        return 1
    
    logging.info('\n所有处理已完成！')
    logging.info('输出目录: %s', args.output)
    logging.info('请将此目录复制到Windows侧使用')
    
    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())

