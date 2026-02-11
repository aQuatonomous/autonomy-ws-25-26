import os
import cv2


def extract_frames(
    video_path: str,
    output_dir: str,
    num_frames: int = 100,
) -> None:
    """
    Extract `num_frames` equally spaced frames from `video_path`
    and save them as PNG images into `output_dir`.

    If the video has fewer than `num_frames` decodable frames,
    frames will be duplicated so that exactly `num_frames` images
    are written.
    """
    if not os.path.isfile(video_path):
        raise FileNotFoundError(f"Video not found: {video_path}")

    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {video_path}")

    # Read all decodable frames sequentially
    frames = []
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            break
        frames.append(frame)

    cap.release()
    total = len(frames)
    if total == 0:
        raise RuntimeError(f"No decodable frames in video: {video_path}")

    # Require enough frames for all requested images (no duplication).
    if total < num_frames:
        raise RuntimeError(
            f"Video has only {total} decodable frames; "
            f"cannot extract {num_frames} distinct images without duplication."
        )

    # Compute evenly spaced, distinct indices in [0, total-1].
    # Use a deterministic step so all indices are unique when total >= num_frames.
    if num_frames == 1:
        indices = [0]
    else:
        step = (total - 1) / float(num_frames - 1)
        indices = [int(round(i * step)) for i in range(num_frames)]

    # Clear out any existing PNGs in the output directory so counts are obvious
    for name in os.listdir(output_dir):
        if name.lower().endswith(".png"):
            try:
                os.remove(os.path.join(output_dir, name))
            except OSError:
                pass

    saved = 0
    for i, idx in enumerate(indices):
        idx = max(0, min(total - 1, idx))
        frame = frames[idx]
        filename = f"frame_{i:03d}.png"
        out_path = os.path.join(output_dir, filename)
        cv2.imwrite(out_path, frame)
        saved += 1

    print(
        f"Video had {total} decodable frames; "
        f"requested {num_frames}, saved {saved} images to '{output_dir}'."
    )


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    video = os.path.join(here, "calibrate.mkv")
    # Folder name based on user's request
    out_dir = os.path.join(here, "calibrate_frames_100")
    extract_frames(video, out_dir, num_frames=100)
