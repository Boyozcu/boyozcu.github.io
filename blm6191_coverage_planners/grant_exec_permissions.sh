#!/bin/bash
# This script automatically grants execute permissions to .sh and .py files in the current directory.

# Find all .sh and .py files in the current directory and grant execute permission.
find . -maxdepth 1 -type f \( -name "*.sh" -o -name "*.py" \) -print0 | while IFS= read -r -d $'\0' file;
do
    if [ -f "$file" ]; then
        echo "Granting execute permission to $file"
        chmod +x "$file"
    else
        echo "Error: $file is not a regular file or does not exist after find."
    fi
done

echo "Finished granting permissions."