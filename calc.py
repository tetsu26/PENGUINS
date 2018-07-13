import math
move_dist = input()
print(move_dist/63)
move_angle = math.asin((move_dist/63)+(1/math.sqrt(2.0)))
move_angle = (move_angle/math.pi)*180-45
print(move_angle)
