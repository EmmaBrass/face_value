import os
import sys
import subprocess

def compile_proto():
    # Get the project root directory
    project_root = os.path.dirname(os.path.abspath(__file__))
    
    # Path to the proto file
    proto_path = os.path.join(project_root, 'grpc_image_streaming', 'proto', 'image_stream.proto')
    
    # Output directory for compiled files
    output_dir = os.path.join(project_root, 'grpc_image_streaming', 'proto')
    
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Compile command
    compile_command = [
        sys.executable, '-m', 'grpc_tools.protoc',
        f'-I{project_root}',
        f'--python_out={output_dir}',
        f'--grpc_python_out={output_dir}',
        proto_path
    ]
    
    try:
        # Run the compilation
        result = subprocess.run(compile_command, capture_output=True, text=True)
        
        # Check for errors
        if result.returncode != 0:
            print("Compilation failed:")
            print(result.stderr)
            return False
        
        # Create __init__.py to make it a package
        init_path = os.path.join(output_dir, '__init__.py')
        with open(init_path, 'w') as f:
            f.write('# Protobuf package\n')
        
        print("Protobuf compilation successful.")
        return True
    
    except Exception as e:
        print(f"An error occurred during compilation: {e}")
        return False

if __name__ == '__main__':
    compile_proto()
