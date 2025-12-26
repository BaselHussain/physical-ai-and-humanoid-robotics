"""
JWT Claims Extractor for Better Auth tokens.

Extracts user information and custom background profile claims from JWT tokens.
Custom claims are stored in the 'https://yourdomain.com/claims' namespace per RFC 7519.
"""

from typing import Dict, Any, Optional
from pydantic import ValidationError

from .models import BackgroundProfile


# Custom claims namespace (matches Better Auth configuration)
CUSTOM_CLAIMS_NAMESPACE = "https://yourdomain.com/claims"


def extract_user_id(payload: Dict[str, Any]) -> str:
    """
    Extract user ID from JWT payload.

    Args:
        payload: Decoded JWT payload

    Returns:
        User ID string

    Raises:
        ValueError: If user ID not found in payload
    """
    user_id = payload.get("id")
    if not user_id:
        raise ValueError("User ID not found in JWT payload")
    return str(user_id)


def extract_user_email(payload: Dict[str, Any]) -> Optional[str]:
    """
    Extract user email from JWT payload.

    Args:
        payload: Decoded JWT payload

    Returns:
        User email string or None if not present
    """
    return payload.get("email")


def extract_user_name(payload: Dict[str, Any]) -> Optional[str]:
    """
    Extract user name from JWT payload.

    Args:
        payload: Decoded JWT payload

    Returns:
        User name string or None if not present
    """
    return payload.get("name")


def extract_background_profile(payload: Dict[str, Any]) -> Optional[BackgroundProfile]:
    """
    Extract user background profile from JWT custom claims.

    Custom claims are expected in the namespace: 'https://yourdomain.com/claims'
    containing:
    - programming_experience: User's programming experience level
    - ros2_familiarity: ROS 2 familiarity level
    - hardware_access: Access to physical hardware

    Args:
        payload: Decoded JWT payload

    Returns:
        BackgroundProfile object if custom claims present and valid, None otherwise

    Example JWT payload:
        {
            "id": "user123",
            "email": "user@example.com",
            "name": "Test User",
            "https://yourdomain.com/claims": {
                "programming_experience": "intermediate",
                "ros2_familiarity": "basic",
                "hardware_access": "simulation"
            }
        }
    """
    # Get custom claims from namespace
    custom_claims = payload.get(CUSTOM_CLAIMS_NAMESPACE)

    if not custom_claims:
        # No custom claims present
        return None

    try:
        # Map Better Auth field names to BackgroundProfile field names
        # Better Auth uses snake_case, which matches our model
        profile_data = {
            'programming_experience': custom_claims.get('programming_experience'),
            'ros2_familiarity': custom_claims.get('ros2_familiarity'),
            'hardware_access': custom_claims.get('hardware_access'),
        }

        # Remove None values
        profile_data = {k: v for k, v in profile_data.items() if v is not None}

        if not profile_data:
            # All fields are None
            return None

        # Validate and create BackgroundProfile
        profile = BackgroundProfile(**profile_data)
        return profile

    except (ValidationError, TypeError, ValueError):
        # Invalid custom claims format or validation error
        return None


def extract_all_user_info(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Extract all user information from JWT payload.

    Convenience function that extracts all standard and custom claims.

    Args:
        payload: Decoded JWT payload

    Returns:
        Dictionary containing:
        - user_id: User ID string
        - email: User email (optional)
        - name: User name (optional)
        - background: BackgroundProfile object (optional)
        - raw_payload: Original JWT payload

    Raises:
        ValueError: If user ID not found
    """
    user_id = extract_user_id(payload)
    email = extract_user_email(payload)
    name = extract_user_name(payload)
    background = extract_background_profile(payload)

    return {
        'user_id': user_id,
        'email': email,
        'name': name,
        'background': background,
        'raw_payload': payload,
    }
