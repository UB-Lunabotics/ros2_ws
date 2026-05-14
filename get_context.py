import os

def collect_workspace_context(output_file='ros2_workspace_context.md'):
    # Define directories to ignore to keep the context clean
    ignore_dirs = {'.git', 'install', 'build', 'log', '__pycache__', '.pytest_cache'}
    # Define file extensions we care about for code context
    valid_extensions = {'.py', '.urdf', '.yaml', '.xml', '.launch.py', '.cpp', '.hpp', 'txt'}

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# ROS 2 Workspace Context\n\n")
        f.write("This file contains the structure and contents of the ROS 2 workspace.\n\n")

        for root, dirs, files in os.walk('.'):
            # Filter out ignored directories
            dirs[:] = [d for d in dirs if d not in ignore_dirs]

            for file in files:
                if any(file.endswith(ext) for ext in valid_extensions):
                    file_path = os.path.join(root, file)
                    f.write(f"## File: {file_path}\n")
                    f.write("```")
                    
                    # Add language hint for markdown
                    ext = os.path.splitext(file)[1]
                    if ext == '.py': f.write('python')
                    elif ext == '.yaml': f.write('yaml')
                    elif ext == '.xml' or ext == '.urdf': f.write('xml')
                    
                    f.write("\n")
                    
                    try:
                        with open(file_path, 'r', encoding='utf-8') as content_file:
                            f.write(content_file.read())
                    except Exception as e:
                        f.write(f"Error reading file: {e}")
                    
                    f.write("\n```\n\n")

    print(f"Successfully generated {output_file}")

if __name__ == '__main__':
    collect_workspace_context()