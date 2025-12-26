"""Test personalization mapping logic"""
from src.auth.models import BackgroundProfile, ProgrammingExperience, ROS2Familiarity, HardwareAccess
from src.auth.personalization import map_background_to_expertise, generate_system_prompt_prefix

# Example: Beginner user
beginner_profile = BackgroundProfile(
    programming_experience=ProgrammingExperience.ZERO_TO_TWO,
    ros2_familiarity=ROS2Familiarity.NONE,
    hardware_access=HardwareAccess.NONE,
)
print("Beginner Profile:")
print(f"  Expertise: {map_background_to_expertise(beginner_profile)}")
print(f"  Prompt prefix: {generate_system_prompt_prefix(beginner_profile)}")
print()

# Example: Advanced user
advanced_profile = BackgroundProfile(
    programming_experience=ProgrammingExperience.TEN_PLUS,
    ros2_familiarity=ROS2Familiarity.ADVANCED,
    hardware_access=HardwareAccess.PHYSICAL_HARDWARE,
)
print("Advanced Profile:")
print(f"  Expertise: {map_background_to_expertise(advanced_profile)}")
print(f"  Prompt prefix: {generate_system_prompt_prefix(advanced_profile)}")
print()

# Example: Intermediate user
intermediate_profile = BackgroundProfile(
    programming_experience=ProgrammingExperience.SIX_TO_TEN,
    ros2_familiarity=ROS2Familiarity.INTERMEDIATE,
    hardware_access=HardwareAccess.SIMULATION_ONLY,
)
print("Intermediate Profile:")
print(f"  Expertise: {map_background_to_expertise(intermediate_profile)}")
print(f"  Prompt prefix: {generate_system_prompt_prefix(intermediate_profile)}")
