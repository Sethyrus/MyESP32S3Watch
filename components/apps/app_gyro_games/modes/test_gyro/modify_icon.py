#!/usr/bin/env python3
"""
Script to modify LVGL icon C-arrays (ARGB8888) to add:
1. Rounded corners with alpha gradient
2. Bottom shadow/darkening effect

Usage:
    python3 modify_icon.py <input_file> [options]

Example:
    python3 modify_icon.py gyro_game_icon.c --radius 28 --shadow 0.2
"""
import re
import math
import argparse
import sys
import os

def parse_args():
    parser = argparse.ArgumentParser(description='Add aesthetic effects to LVGL C-array icons.')
    parser.add_argument('input_file', help='Path to the input C file containing the image array')
    parser.add_argument('--output', '-o', help='Path to output file (default: overwrite input)')
    parser.add_argument('--width', type=int, default=112, help='Image width (default: 112)')
    parser.add_argument('--height', type=int, default=112, help='Image height (default: 112)')
    parser.add_argument('--radius', type=float, default=28.0, help='Corner radius in pixels (default: 28.0)')
    parser.add_argument('--shadow', type=float, default=0.20, help='Maximum shadow darkening factor (0.0-1.0, default: 0.2)')
    parser.add_argument('--backup', action='store_true', help='Create a backup of the input file (.bak)')
    return parser.parse_args()

def parse_c_file(filepath):
    """Read file and extract pixel data and variable name"""
    with open(filepath, 'r') as f:
        content = f.read()

    # Generic regex to find the uint8_t array
    # Looks for: const ... uint8_t ... variable_name[] = { ... }
    # Capture group 1: variable name
    # Capture group 2: array content
    pattern = r'uint8_t\s+([a-zA-Z0-9_]+)\[\]\s*=\s*\{([^}]+)\}'
    match = re.search(pattern, content, re.DOTALL)

    if not match:
        raise ValueError(f"Could not find valid uint8_t array definition in {filepath}")

    var_name = match.group(1)
    data_str = match.group(2)

    # Extract all hex values
    hex_values = re.findall(r'0x[0-9a-fA-F]{2}', data_str)

    return [int(h, 16) for h in hex_values], content, var_name

def distance_to_corner(x, y, width, height, radius):
    """Calculate if a pixel is outside the rounded corner and by how much"""
    # Check each corner
    corners = [
        (radius, radius),                    # top-left
        (width - radius - 1, radius),        # top-right
        (radius, height - radius - 1),       # bottom-left
        (width - radius - 1, height - radius - 1)  # bottom-right
    ]

    for cx, cy in corners:
        # Check if we're in the corner region
        in_corner_x = (x < radius and cx == radius) or (x > width - radius - 1 and cx == width - radius - 1)
        in_corner_y = (y < radius and cy == radius) or (y > height - radius - 1 and cy == height - radius - 1)

        if in_corner_x and in_corner_y:
            # Calculate distance from corner center
            dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            if dist > radius:
                # Completely outside
                return 1.0
            elif dist > radius - 1.5:
                # Anti-aliasing zone
                return (dist - (radius - 1.5)) / 1.5

    return 0.0  # Inside the rounded rect

def get_shadow_factor(y, height, max_darkening):
    """Calculate darkening factor for bottom shadow effect"""
    # Start shadow from 70% of height
    shadow_start = int(height * 0.7)
    if y < shadow_start:
        return 1.0

    # Gradual darkening towards bottom
    progress = (y - shadow_start) / (height - shadow_start)
    return 1.0 - (progress * max_darkening)

def apply_modifications(pixels, width, height, radius, max_shadow):
    """Apply rounded corners and shadow to pixel data"""
    if len(pixels) != width * height * 4:
        raise ValueError(f"Pixel count {len(pixels)} does not match dimensions {width}x{height} (expected {width*height*4} bytes)")

    modified = list(pixels)

    for y in range(height):
        for x in range(width):
            idx = (y * width + x) * 4  # BGRA format

            # Get current pixel values (BGRA)
            b = modified[idx]
            g = modified[idx + 1]
            r = modified[idx + 2]
            a = modified[idx + 3]

            # Apply corner rounding
            corner_dist = distance_to_corner(x, y, width, height, radius)
            if corner_dist >= 1.0:
                # Fully transparent
                modified[idx] = 0
                modified[idx + 1] = 0
                modified[idx + 2] = 0
                modified[idx + 3] = 0
            elif corner_dist > 0:
                # Anti-aliased edge
                new_alpha = int(a * (1.0 - corner_dist))
                modified[idx + 3] = new_alpha
            else:
                # Apply shadow effect (only to fully visible pixels inside curve)
                shadow_factor = get_shadow_factor(y, height, max_shadow)
                if shadow_factor < 1.0:
                    modified[idx] = int(b * shadow_factor)
                    modified[idx + 1] = int(g * shadow_factor)
                    modified[idx + 2] = int(r * shadow_factor)

    return modified

def format_output(pixels, original_content, width):
    """Format the modified pixels back into C code"""
    # Generate new array content
    lines = []
    line_pixels = []

    for i in range(0, len(pixels), 4):
        # New line every 'width' pixels (for readability)
        if i > 0 and (i // 4) % width == 0:
            lines.append("  " + ", ".join(line_pixels) + ",")
            line_pixels = []

        b, g, r, a = pixels[i:i+4]
        line_pixels.append(f"0x{b:02x}, 0x{g:02x}, 0x{r:02x}, 0x{a:02x}")

    if line_pixels:
        lines.append("  " + ", ".join(line_pixels) + ",")

    array_content = "\n".join(lines)

    # Replace the array content in original file
    # We reconstruct the regex to match specifically the array we found earlier or any similar one
    new_content = re.sub(
        r'(uint8_t\s+[a-zA-Z0-9_]+\[\]\s*=\s*\{)[^}]+(\})',
        r'\1\n' + array_content + '\n' + r'\2',
        original_content,
        flags=re.DOTALL
    )

    return new_content

def main():
    args = parse_args()

    if not os.path.exists(args.input_file):
        print(f"Error: Input file '{args.input_file}' not found.")
        sys.exit(1)

    print(f"Processing {args.input_file}...")

    try:
        pixels, original_content, var_name = parse_c_file(args.input_file)
        print(f"Found array: {var_name}")
        print(f"Read {len(pixels)} bytes ({len(pixels) // 4} pixels)")

        modified_pixels = apply_modifications(
            pixels,
            args.width,
            args.height,
            args.radius,
            args.shadow
        )
        print("Applied rounded corners and shadow effect.")

        new_content = format_output(modified_pixels, original_content, args.width)

        output_file = args.output if args.output else args.input_file

        if args.backup and output_file == args.input_file:
            bak_file = args.input_file + ".bak"
            with open(bak_file, 'w') as f:
                f.write(original_content)
            print(f"Backup created: {bak_file}")

        with open(output_file, 'w') as f:
            f.write(new_content)

        print(f"Successfully wrote to {output_file}")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
