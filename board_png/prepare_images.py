import os
from pathlib import Path
from datetime import datetime
from PIL import Image, ImageOps
from PIL.ExifTags import TAGS


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"}

def get_exif_timestamp(image_path):
    try:
        img = Image.open(image_path)
        exif_data = img._getexif()
        if exif_data is None:
            return None

        for tag_id, value in exif_data.items():
            tag = TAGS.get(tag_id, tag_id)
            if tag == "DateTimeOriginal":
                return datetime.strptime(value, "%Y:%m:%d %H:%M:%S")
    except Exception:
        pass
    return None

def get_timestamp(image_path):
    # Try EXIF first
    ts = get_exif_timestamp(image_path)
    if ts is not None:
        return ts

    # Fallback to file modification time
    return datetime.fromtimestamp(os.path.getmtime(image_path))


def get_images(directory):
    directory = Path(directory)
    return [p for p in directory.iterdir() if p.suffix.lower() in IMAGE_EXTENSIONS]


def get_smallest_image_size(images):
    image_sizes = []
    for image_path in images:
        with Image.open(image_path) as img:
            img = ImageOps.exif_transpose(img)
            width, height = img.size
        image_sizes.append((width * height, width, height, image_path))

    _, width, height, image_path = min(image_sizes, key=lambda item: item[0])
    print(f"Smallest image: {image_path.name} ({width}x{height})")
    return width, height


def resize_images_to_size(images, target_size):
    target_width, target_height = target_size
    resized_count = 0

    for image_path in images:
        with Image.open(image_path) as img:
            img = ImageOps.exif_transpose(img)

            if img.size == target_size:
                continue

            resized = img.resize((target_width, target_height), Image.Resampling.LANCZOS)
            resized.save(image_path)
            resized_count += 1

    print(f"Resized {resized_count} images to {target_width}x{target_height}.")


def rename_images_by_timestamp(directory):
    directory = Path(directory)

    images = get_images(directory)

    if not images:
        print("No images found.")
        return

    # Extract timestamps
    image_data = []
    for img in images:
        ts = get_timestamp(img)
        image_data.append((img, ts))

    # Sort by timestamp
    image_data.sort(key=lambda x: x[1])

    # First pass: rename to temp to avoid overwrite conflicts
    temp_paths = []
    for i, (img, _) in enumerate(image_data):
        temp_path = directory / f"temp_{i:06d}{img.suffix.lower()}"
        img.rename(temp_path)
        temp_paths.append(temp_path)

    # Second pass: final renaming
    for i, temp_path in enumerate(temp_paths):
        new_name = directory / f"{i+1:06d}{temp_path.suffix.lower()}"
        temp_path.rename(new_name)

    print(f"Renamed {len(temp_paths)} images successfully.")


def prepare_images(directory):
    directory = Path(directory)
    images = get_images(directory)

    if not images:
        print("No images found.")
        return

    target_size = get_smallest_image_size(images)
    resize_images_to_size(images, target_size)
    rename_images_by_timestamp(directory)


if __name__ == "__main__":
    prepare_images("data/")
