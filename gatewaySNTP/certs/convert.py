import os
import shutil
import fnmatch

# Define the patterns for certificate and key files
patterns = {
    "AmazonRootCA1.pem": "ca-cert.pem",  # This file has a fixed name
    "*-certificate.pem.crt": "client-cert.pem",  # Pattern to match client certificate
    "*-private.pem.key": "private-key.pem"       # Pattern to match private key
}

def find_matching_file(pattern, directory):
    """Find a file in the directory that matches the pattern."""
    for file in os.listdir(directory):
        if fnmatch.fnmatch(file, pattern):
            return file
    return None

def convert_and_format(input_path, output_path):
    """Convert file by copying the content from input to output with formatting."""
    if not os.path.exists(input_path):
        print(f"File not found: {input_path}")
        return

    try:
        with open(input_path, 'r') as cert_file:
            cert_content = cert_file.readlines()

        with open(output_path, 'w') as output_file:
            # Write the formatted content to the output file
            for line in cert_content:
                output_file.write(f'"{line.rstrip()}\\n"\n')  # Add " at start and \n at end

        print(f"Converted: {input_path} -> {output_path}")
    except Exception as e:
        print(f"Error processing {input_path}: {e}")

# Directory where your certificates are located
cert_dir = "certs"

# Process each pattern and find matching files
for pattern, output_file in patterns.items():
    if pattern == "AmazonRootCA1.pem":
        input_file = os.path.join(cert_dir, pattern)
    else:
        # For patterns, find the matching file in the directory
        input_file = find_matching_file(pattern, cert_dir)
        if input_file:
            input_file = os.path.join(cert_dir, input_file)
        else:
            print(f"No file found matching pattern: {pattern}")
            continue
    
    output_path = os.path.join(cert_dir, output_file)
    convert_and_format(input_file, output_path)
