"""
Personalization Mapping Logic

Maps user BackgroundProfile to expertise levels and RAG agent prompt adjustments.
Implements personalization rules from spec.md SC-007, SC-008.
"""

from enum import Enum
from typing import Dict, Optional
from .models import BackgroundProfile, ProgrammingExperience, ROS2Familiarity, HardwareAccess


class ExpertiseLevel(str, Enum):
    """Derived expertise level for RAG agent personalization"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    DEFAULT = "default_intermediate"  # Fallback if background unavailable


def map_background_to_expertise(profile: BackgroundProfile) -> ExpertiseLevel:
    """
    Map BackgroundProfile to ExpertiseLevel.

    Mapping Rules (from data-model.md):
    - Beginner: 0-2 years programming AND (ROS2=None OR ROS2=Beginner)
    - Intermediate: 3-10 years programming OR ROS2=Intermediate
    - Advanced: 10+ years programming AND ROS2=Advanced

    Edge case: 10+ years programming but ROS2=None/Beginner → Intermediate
    (experienced programmer but new to ROS 2)

    Args:
        profile: User's BackgroundProfile

    Returns:
        ExpertiseLevel (beginner, intermediate, advanced)
    """
    prog_exp = profile.programming_experience
    ros2_fam = profile.ros2_familiarity

    # Beginner: 0-2 years prog AND (ROS2=None OR ROS2=Beginner)
    if prog_exp == ProgrammingExperience.ZERO_TO_TWO:
        if ros2_fam in [ROS2Familiarity.NONE, ROS2Familiarity.BEGINNER]:
            return ExpertiseLevel.BEGINNER

    # Beginner: 3-5 years prog AND (ROS2=None OR ROS2=Beginner)
    if prog_exp == ProgrammingExperience.THREE_TO_FIVE:
        if ros2_fam in [ROS2Familiarity.NONE, ROS2Familiarity.BEGINNER]:
            return ExpertiseLevel.BEGINNER

    # Advanced: 10+ years prog AND ROS2=Advanced
    if prog_exp == ProgrammingExperience.TEN_PLUS and ros2_fam == ROS2Familiarity.ADVANCED:
        return ExpertiseLevel.ADVANCED

    # Intermediate: Everything else
    # - 3-5 years with ROS2=Intermediate
    # - 6-10 years (any ROS2 level)
    # - 10+ years with ROS2=Intermediate, Beginner, or None
    return ExpertiseLevel.INTERMEDIATE


def get_prompt_adjustments(profile: BackgroundProfile) -> Dict[str, str]:
    """
    Generate RAG agent system prompt adjustments based on expertise level.

    Returns dict with:
    - expertise_level: beginner/intermediate/advanced
    - language_style: Description of language complexity
    - explanation_depth: Level of technical detail
    - code_examples: Whether to include code examples
    - hardware_context: Hardware-specific guidance

    Args:
        profile: User's BackgroundProfile

    Returns:
        Dict with prompt adjustment parameters
    """
    expertise = map_background_to_expertise(profile)
    hardware = profile.hardware_access

    # Base adjustments by expertise level
    if expertise == ExpertiseLevel.BEGINNER:
        adjustments = {
            "expertise_level": "beginner",
            "language_style": "simple, step-by-step",
            "explanation_depth": "detailed explanations with analogies, avoid jargon",
            "code_examples": "minimal, well-commented code only when necessary",
            "technical_terms": "introduce technical terms gradually with definitions",
        }
    elif expertise == ExpertiseLevel.ADVANCED:
        adjustments = {
            "expertise_level": "advanced",
            "language_style": "technical and precise",
            "explanation_depth": "in-depth technical details, architectural considerations",
            "code_examples": "code-heavy responses with advanced patterns",
            "technical_terms": "use technical terminology freely, assume familiarity",
        }
    else:  # Intermediate
        adjustments = {
            "expertise_level": "intermediate",
            "language_style": "balanced technical depth with clear explanations",
            "explanation_depth": "moderate detail, explain complex concepts clearly",
            "code_examples": "practical code examples with explanations",
            "technical_terms": "use technical terms with brief context",
        }

    # Add hardware-specific context
    if hardware == HardwareAccess.NONE:
        adjustments["hardware_context"] = "general conceptual explanations, no hardware assumptions"
    elif hardware == HardwareAccess.SIMULATION_ONLY:
        adjustments["hardware_context"] = "simulation-focused guidance (Gazebo, Isaac Sim), virtual testing"
    else:  # Physical hardware
        adjustments["hardware_context"] = "hardware-specific advice, sensor integration, physical deployment considerations"

    return adjustments


def generate_system_prompt_prefix(profile: Optional[BackgroundProfile]) -> str:
    """
    Generate system prompt prefix for RAG agent based on user background.

    This prefix is prepended to the default RAG agent system prompt to personalize responses.

    Args:
        profile: User's BackgroundProfile (None for guest users)

    Returns:
        System prompt prefix string
    """
    if profile is None:
        # Guest user - use default intermediate level
        return (
            "You are a helpful robotics assistant. "
            "Provide balanced technical explanations suitable for intermediate learners. "
            "Include practical code examples when relevant."
        )

    adjustments = get_prompt_adjustments(profile)
    expertise = adjustments["expertise_level"]

    # Build personalized prompt prefix
    prompt_parts = [
        f"You are a helpful robotics assistant tailored to a user with {expertise}-level expertise."
    ]

    # Language style guidance
    if expertise == "beginner":
        prompt_parts.append(
            "Use simple, step-by-step explanations. Avoid jargon or introduce technical terms gradually with definitions. "
            "Provide detailed explanations with analogies to help build understanding."
        )
    elif expertise == "advanced":
        prompt_parts.append(
            "Use technical terminology freely and provide in-depth technical details. "
            "Include architectural considerations, design patterns, and code-heavy responses."
        )
    else:  # intermediate
        prompt_parts.append(
            "Provide balanced technical depth with clear explanations. "
            "Use technical terms with brief context and include practical code examples."
        )

    # Hardware context guidance
    hardware_guidance = adjustments["hardware_context"]
    prompt_parts.append(f"Hardware context: {hardware_guidance}")

    # Code examples guidance
    code_guidance = adjustments["code_examples"]
    prompt_parts.append(f"Code examples: {code_guidance}")

    return " ".join(prompt_parts)


# ============================================================================
# Example Usage (for testing)
# ============================================================================

if __name__ == "__main__":
    # Example: Beginner user
    beginner_profile = BackgroundProfile(
        programming_experience=ProgrammingExperience.ZERO_TO_TWO,
        ros2_familiarity=ROS2Familiarity.NONE,
        hardware_access=HardwareAccess.NONE,
    )
    print("Beginner Profile:")
    print(f"  Expertise: {map_background_to_expertise(beginner_profile)}")
    print(f"  Prompt prefix: {generate_system_prompt_prefix(beginner_profile)[:100]}...")
    print()

    # Example: Advanced user
    advanced_profile = BackgroundProfile(
        programming_experience=ProgrammingExperience.TEN_PLUS,
        ros2_familiarity=ROS2Familiarity.ADVANCED,
        hardware_access=HardwareAccess.PHYSICAL_HARDWARE,
    )
    print("Advanced Profile:")
    print(f"  Expertise: {map_background_to_expertise(advanced_profile)}")
    print(f"  Prompt prefix: {generate_system_prompt_prefix(advanced_profile)[:100]}...")
    print()

    # Example: Intermediate user
    intermediate_profile = BackgroundProfile(
        programming_experience=ProgrammingExperience.SIX_TO_TEN,
        ros2_familiarity=ROS2Familiarity.INTERMEDIATE,
        hardware_access=HardwareAccess.SIMULATION_ONLY,
    )
    print("Intermediate Profile:")
    print(f"  Expertise: {map_background_to_expertise(intermediate_profile)}")
    print(f"  Prompt prefix: {generate_system_prompt_prefix(intermediate_profile)[:100]}...")
