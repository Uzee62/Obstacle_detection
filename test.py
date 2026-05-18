from maritime_perception.sensors.lidar.driver import RPLidarDriver

driver = RPLidarDriver(port="/dev/ttyUSB0", baudrate=1_000_000)

try:
    driver.connect()
    print("Connected. Reading one scan...")
    scan = driver.read_scan()
    print(f"Got {len(scan)} points")
    print("First 5 points:")
    for pt in scan.points[:5]:
        print(f"  angle={pt.angle_deg:.1f}°  dist={pt.distance_m:.3f}m  quality={pt.quality}")
finally:
    driver.disconnect()
    print("Disconnected")