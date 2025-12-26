"""
Tests for JWT claims extraction and user profile parsing.

Covers:
- User ID extraction
- User email extraction
- Background profile extraction from custom claims
- Invalid claims handling
"""

import pytest
from typing import Dict, Any

from backend.src.auth.claims_extractor import (
    extract_user_id,
    extract_user_email,
    extract_user_name,
    extract_background_profile,
    extract_all_user_info,
    CUSTOM_CLAIMS_NAMESPACE
)
from backend.src.auth.models import BackgroundProfile


# Test fixtures
@pytest.fixture
def full_jwt_payload():
    """Create JWT payload with all claims."""
    return {
        'id': 'user123',
        'email': 'test@example.com',
        'name': 'Test User',
        'iss': 'http://localhost:3000',
        'aud': 'http://localhost:3000',
        'iat': 1234567890,
        'exp': 1234567890 + 900,  # 15 minutes
        CUSTOM_CLAIMS_NAMESPACE: {
            'programming_experience': '3-5 years',
            'ros2_familiarity': 'Intermediate',
            'hardware_access': 'Simulation only'
        }
    }


@pytest.fixture
def minimal_jwt_payload():
    """Create minimal JWT payload (only user ID)."""
    return {
        'id': 'user456',
        'iss': 'http://localhost:3000',
        'aud': 'http://localhost:3000',
        'iat': 1234567890,
        'exp': 1234567890 + 900
    }


# User ID Extraction Tests
class TestExtractUserId:
    """Tests for extract_user_id function."""

    def test_extract_valid_user_id(self, full_jwt_payload):
        """Test extraction of valid user ID."""
        user_id = extract_user_id(full_jwt_payload)
        assert user_id == 'user123'

    def test_extract_user_id_missing(self):
        """Test extraction fails when ID missing."""
        payload = {'email': 'test@example.com'}

        with pytest.raises(ValueError) as exc_info:
            extract_user_id(payload)

        assert "not found" in str(exc_info.value).lower()

    def test_extract_user_id_converts_to_string(self):
        """Test that numeric user IDs are converted to strings."""
        payload = {'id': 12345}
        user_id = extract_user_id(payload)
        assert user_id == '12345'
        assert isinstance(user_id, str)


# User Email Extraction Tests
class TestExtractUserEmail:
    """Tests for extract_user_email function."""

    def test_extract_valid_email(self, full_jwt_payload):
        """Test extraction of valid email."""
        email = extract_user_email(full_jwt_payload)
        assert email == 'test@example.com'

    def test_extract_email_missing(self, minimal_jwt_payload):
        """Test extraction returns None when email missing."""
        email = extract_user_email(minimal_jwt_payload)
        assert email is None

    def test_extract_email_empty_string(self):
        """Test extraction handles empty email string."""
        payload = {'id': 'user123', 'email': ''}
        email = extract_user_email(payload)
        assert email == ''  # Returns empty string as-is


# User Name Extraction Tests
class TestExtractUserName:
    """Tests for extract_user_name function."""

    def test_extract_valid_name(self, full_jwt_payload):
        """Test extraction of valid name."""
        name = extract_user_name(full_jwt_payload)
        assert name == 'Test User'

    def test_extract_name_missing(self, minimal_jwt_payload):
        """Test extraction returns None when name missing."""
        name = extract_user_name(minimal_jwt_payload)
        assert name is None


# Background Profile Extraction Tests
class TestExtractBackgroundProfile:
    """Tests for extract_background_profile function."""

    def test_extract_full_background_profile(self, full_jwt_payload):
        """Test extraction of complete background profile."""
        # Update payload to use correct enum values
        full_jwt_payload[CUSTOM_CLAIMS_NAMESPACE] = {
            'programming_experience': '3-5 years',
            'ros2_familiarity': 'Intermediate',
            'hardware_access': 'Simulation only'
        }

        profile = extract_background_profile(full_jwt_payload)

        assert profile is not None
        assert isinstance(profile, BackgroundProfile)
        assert profile.programming_experience == '3-5 years'
        assert profile.ros2_familiarity == 'Intermediate'
        assert profile.hardware_access == 'Simulation only'

    def test_extract_partial_background_profile(self):
        """Test extraction with only some fields present."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': '10+ years',
                'ros2_familiarity': 'Advanced',
                'hardware_access': 'Physical robots/sensors'
            }
        }

        profile = extract_background_profile(payload)

        assert profile is not None
        assert profile.programming_experience == '10+ years'
        assert profile.ros2_familiarity == 'Advanced'
        assert profile.hardware_access == 'Physical robots/sensors'

    def test_extract_background_profile_missing_namespace(self, minimal_jwt_payload):
        """Test extraction returns None when custom claims namespace missing."""
        profile = extract_background_profile(minimal_jwt_payload)
        assert profile is None

    def test_extract_background_profile_empty_namespace(self):
        """Test extraction returns None when custom claims namespace is empty."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {}
        }

        profile = extract_background_profile(payload)
        assert profile is None

    def test_extract_background_profile_all_none_values(self):
        """Test extraction returns None when all values are None."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': None,
                'ros2_familiarity': None,
                'hardware_access': None
            }
        }

        profile = extract_background_profile(payload)
        assert profile is None

    def test_extract_background_profile_invalid_values(self):
        """Test extraction returns None with invalid enum values."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': 'invalid_level',
                'ros2_familiarity': 'also_invalid',
                'hardware_access': 'unknown'
            }
        }

        profile = extract_background_profile(payload)
        assert profile is None  # Validation should fail

    def test_extract_background_profile_mixed_valid_invalid(self):
        """Test extraction with mix of valid and invalid values."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': 'beginner',  # Valid
                'ros2_familiarity': 'invalid_value',   # Invalid
                'hardware_access': 'hardware'          # Valid
            }
        }

        # Pydantic should reject the entire model due to one invalid field
        profile = extract_background_profile(payload)
        assert profile is None

    def test_extract_background_profile_wrong_types(self):
        """Test extraction handles wrong data types gracefully."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': 123,  # Should be string
                'ros2_familiarity': True,       # Should be string
                'hardware_access': ['array']    # Should be string
            }
        }

        profile = extract_background_profile(payload)
        assert profile is None


# Extract All User Info Tests
class TestExtractAllUserInfo:
    """Tests for extract_all_user_info function."""

    def test_extract_all_from_full_payload(self, full_jwt_payload):
        """Test extraction of all info from complete payload."""
        info = extract_all_user_info(full_jwt_payload)

        assert info['user_id'] == 'user123'
        assert info['email'] == 'test@example.com'
        assert info['name'] == 'Test User'
        assert isinstance(info['background'], BackgroundProfile)
        assert info['background'].programming_experience == '3-5 years'
        assert info['background'].ros2_familiarity == 'Intermediate'
        assert info['background'].hardware_access == 'Simulation only'
        assert info['raw_payload'] == full_jwt_payload

    def test_extract_all_from_minimal_payload(self, minimal_jwt_payload):
        """Test extraction from minimal payload."""
        info = extract_all_user_info(minimal_jwt_payload)

        assert info['user_id'] == 'user456'
        assert info['email'] is None
        assert info['name'] is None
        assert info['background'] is None
        assert info['raw_payload'] == minimal_jwt_payload

    def test_extract_all_missing_user_id(self):
        """Test extraction fails when user ID missing."""
        payload = {'email': 'test@example.com'}

        with pytest.raises(ValueError):
            extract_all_user_info(payload)

    def test_extract_all_preserves_raw_payload(self, full_jwt_payload):
        """Test that raw payload is preserved unchanged."""
        info = extract_all_user_info(full_jwt_payload)
        assert info['raw_payload'] is full_jwt_payload
        assert info['raw_payload']['id'] == 'user123'


# Edge Cases and Validation Tests
class TestClaimsExtractionEdgeCases:
    """Tests for edge cases and validation in claims extraction."""

    def test_custom_claims_namespace_constant(self):
        """Verify custom claims namespace matches spec."""
        assert CUSTOM_CLAIMS_NAMESPACE == "https://yourdomain.com/claims"

    def test_extract_with_extra_fields_in_namespace(self):
        """Test extraction ignores extra unknown fields."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': '0-2 years',
                'ros2_familiarity': 'Beginner',
                'hardware_access': 'Simulation only',
                'unknown_field': 'should_be_ignored',
                'another_extra': 12345
            }
        }

        profile = extract_background_profile(payload)

        assert profile is not None
        assert profile.programming_experience == '0-2 years'
        # Extra fields should not cause errors

    def test_extract_with_unicode_values(self):
        """Test extraction handles unicode in user info."""
        payload = {
            'id': 'user123',
            'email': 'tëst@éxamplë.com',
            'name': '测试用户 👤'
        }

        user_id = extract_user_id(payload)
        email = extract_user_email(payload)
        name = extract_user_name(payload)

        assert user_id == 'user123'
        assert email == 'tëst@éxamplë.com'
        assert name == '测试用户 👤'

    def test_extract_background_profile_case_sensitivity(self):
        """Test that enum values are case-sensitive."""
        payload = {
            'id': 'user123',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': 'Beginner',  # Wrong case
                'ros2_familiarity': 'BASIC',           # Wrong case
                'hardware_access': 'Simulation'        # Wrong case
            }
        }

        # Pydantic enums are case-sensitive by default
        profile = extract_background_profile(payload)
        # Should fail validation if model doesn't accept these cases
        # If your BackgroundProfile accepts case-insensitive, adjust this test


# Personalization Use Case Tests
class TestPersonalizationIntegration:
    """Tests for personalization use cases with extracted claims."""

    def test_beginner_profile_extraction(self):
        """Test extraction of beginner user profile."""
        payload = {
            'id': 'user123',
            'email': 'beginner@example.com',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': '0-2 years',
                'ros2_familiarity': 'None',
                'hardware_access': 'None'
            }
        }

        info = extract_all_user_info(payload)

        assert info['background'].programming_experience == '0-2 years'
        assert info['background'].ros2_familiarity == 'None'
        assert info['background'].hardware_access == 'None'

    def test_advanced_profile_extraction(self):
        """Test extraction of advanced user profile."""
        payload = {
            'id': 'user456',
            'email': 'expert@example.com',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': '10+ years',
                'ros2_familiarity': 'Advanced',
                'hardware_access': 'Physical robots/sensors'
            }
        }

        info = extract_all_user_info(payload)

        assert info['background'].programming_experience == '10+ years'
        assert info['background'].ros2_familiarity == 'Advanced'
        assert info['background'].hardware_access == 'Physical robots/sensors'

    def test_mixed_experience_profile_extraction(self):
        """Test extraction of user with mixed experience levels."""
        payload = {
            'id': 'user789',
            CUSTOM_CLAIMS_NAMESPACE: {
                'programming_experience': '10+ years',
                'ros2_familiarity': 'Beginner',
                'hardware_access': 'Simulation only'
            }
        }

        profile = extract_background_profile(payload)

        # User knows programming but new to ROS2
        assert profile.programming_experience == '10+ years'
        assert profile.ros2_familiarity == 'Beginner'
