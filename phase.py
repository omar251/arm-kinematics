def phase_shift(angle):
    shifted_angle = (angle + 90) % 180
    return shifted_angle
print(phase_shift(0))    # Output: 90
print(phase_shift(45))   # Output: 135
print(phase_shift(90))   # Output: 0
print(phase_shift(135))  # Output: 45
print(phase_shift(180))  # Output: 90
def convert_to_positive(angle):
    if angle < 0:
        return angle + 180
    return angle
print(convert_to_positive(-180))   # Output: 0
print(convert_to_positive(-90))    # Output: 90
print(convert_to_positive(0))      # Output: 0
print(convert_to_positive(90))     # Output: 90
print(convert_to_positive(180))    # Output: 180


# MODULUS = 180

# def phase_shift(angle):
#     shifted_angle = (angle + 90) % MODULUS
#     return shifted_angle

# def convert_to_positive(angle):
#     if angle < 0:
#         return angle + MODULUS
#     return angle
# def angle_operation(angle, operation):
#     if operation == "phase_shift":
#         return (angle + 90) % MODULUS
#     elif operation == "convert_to_positive":
#         if angle < 0:
#             return angle + MODULUS
#         return angle

# print(angle_operation(0, "phase_shift"))    # Output: 90
# print(angle_operation(45, "phase_shift"))   # Output: 135
# print(angle_operation(90, "phase_shift"))   # Output: 0
# print(angle_operation(135, "phase_shift"))  # Output: 45
# print(angle_operation(180, "phase_shift"))  # Output: 90
# print(angle_operation(-180, "convert_to_positive"))   # Output: 0
# print(angle_operation(-90, "convert_to_positive"))    # Output: 90
# print(angle_operation(0, "convert_to_positive"))      # Output: 0
# print(angle_operation(90, "convert_to_positive"))     # Output: 90
# print(angle_operation(180, "convert_to_positive"))    # Output: 180