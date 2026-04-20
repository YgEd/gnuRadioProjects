#!/bin/bash

if [ ! -f "requirements.txt" ]; then
    echo "Error: requirements.txt not found"
    exit 1
fi

while IFS= read -r package || [ -n "$package" ]; do
    # Skip empty lines and comments
    [[ -z "$package" || "$package" == \#* ]] && continue

    echo "Installing: $package"
    if pip install "$package" -q 2>/dev/null; then
        echo "  ✓ Installed: $package"
    else
        echo "  ✗ Skipped (unknown/failed): $package"
    fi

done < requirements.txt
