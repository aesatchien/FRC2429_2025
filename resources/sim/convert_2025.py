from PIL import Image

# Load the image
image_path = "2025-field.png"  # Replace with your image path
img = Image.open(image_path)
pixels = img.load()

# Define the points where crosses will be drawn
points = [(534, 291), (3466, 1638)]

# Define the size of the cross (number of pixels in each direction)
cross_size = 5

# Draw a cross at each specified point
for x, y in points:
    for dx in range(-cross_size, cross_size + 1):
        if 0 <= x + dx < img.width:
            pixels[x + dx, y] = (255, 255, 255, 255) if img.mode == 'RGBA' else (255, 255, 255)
    for dy in range(-cross_size, cross_size + 1):
        if 0 <= y + dy < img.height:
            pixels[x, y + dy] = (255, 255, 255, 255) if img.mode == 'RGBA' else (255, 255, 255)

# Save the edited image in a lossless format
output_path = "2025-field-marked.png"  # Replace with your desired save path
img.save(output_path, format="PNG")

print(f"Edited image saved at {output_path}")
