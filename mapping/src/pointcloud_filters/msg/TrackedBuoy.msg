# Persistent tracked buoy with stable ID and data fusion from LiDAR + vision

uint32 id                           # Unique persistent ID (starts at 0, increments)
float32 range                       # Distance from sensor (meters)
float32 bearing                     # Angle from sensor (radians)
float32 x                           # Cartesian x position (meters)
float32 y                           # Cartesian y position (meters)
float32 z_mean                      # Average z height (meters)
string color                        # Color from vision: "red", "green", "yellow", "unknown"
float32 confidence                  # Detection confidence (0.0 to 1.0)
uint32 observation_count            # Number of times this buoy has been detected
builtin_interfaces/Time first_seen  # Timestamp when first detected
builtin_interfaces/Time last_seen   # Timestamp of most recent detection
