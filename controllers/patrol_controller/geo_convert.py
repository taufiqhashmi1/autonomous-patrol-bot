import math

R = 6378137  

def latlon_to_xy(lat, lon, lat0, lon0):
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = R * dlon * math.cos(math.radians(lat0))
    y = R * dlat
    return x, y

def convert_to_local(latlon_points, arena_size=4.0):
    if not latlon_points:
        return []

    origin = latlon_points[0]
    raw_waypoints = []

    for lat, lon in latlon_points:
        x, y = latlon_to_xy(lat, lon, origin[0], origin[1])
        raw_waypoints.append((x, y))

    xs = [pt[0] for pt in raw_waypoints]
    ys = [pt[1] for pt in raw_waypoints]
    
    cx, cy = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    max_dim = max(max(xs) - min(xs), max(ys) - min(ys))
    
    if max_dim == 0:
        return [(0.0, 0.0) for _ in raw_waypoints]

    scale = arena_size / max_dim

    return [((x - cx) * scale, (y - cy) * scale) for x, y in raw_waypoints]