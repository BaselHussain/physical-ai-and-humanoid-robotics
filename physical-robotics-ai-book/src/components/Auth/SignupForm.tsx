/**
 * SignupForm Component
 *
 * Registration form with email, password, and background profile collection.
 * Maps to User Story 1 (New User Registration) from spec.md
 */

import React, { useState } from 'react';
import styles from './Auth.module.css';

interface BackgroundProfile {
  programming_experience: string;
  ros2_familiarity: string;
  hardware_access: string;
}

interface SignupFormProps {
  onSubmit: (email: string, password: string, name: string, background: BackgroundProfile) => Promise<void>;
  onSwitchToSignin: () => void;
}

export default function SignupForm({ onSubmit, onSwitchToSignin }: SignupFormProps) {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [progExp, setProgExp] = useState('');
  const [ros2Fam, setRos2Fam] = useState('');
  const [hardware, setHardware] = useState('');

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Validation
  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    // Required field validation
    if (!name.trim()) {
      newErrors.name = 'Name is required';
    } else if (name.trim().length < 2) {
      newErrors.name = 'Name must be at least 2 characters';
    }

    if (!email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!validateEmail(email)) {
      newErrors.email = 'Invalid email format';
    }

    if (!password.trim()) {
      newErrors.password = 'Password is required';
    } else if (password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    }

    if (!progExp) {
      newErrors.progExp = 'Programming experience is required';
    }

    if (!ros2Fam) {
      newErrors.ros2Fam = 'ROS 2 familiarity is required';
    }

    if (!hardware) {
      newErrors.hardware = 'Hardware access is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsSubmitting(true);
    setErrors({});

    try {
      const background: BackgroundProfile = {
        programming_experience: progExp,
        ros2_familiarity: ros2Fam,
        hardware_access: hardware,
      };

      // Call updated signup with name parameter
      await onSubmit(email, password, name, background);
    } catch (error: any) {
      setErrors({ submit: error.message || 'Signup failed. Please try again.' });
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className={styles.authForm}>
      <h2>Create Your Account</h2>
      <p className={styles.subtitle}>
        Join to get personalized robotics assistance based on your experience
      </p>

      <form onSubmit={handleSubmit}>
        {/* Name Field */}
        <div className={styles.formGroup}>
          <label htmlFor="name">Full Name *</label>
          <input
            type="text"
            id="name"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="John Doe"
            disabled={isSubmitting}
            className={errors.name ? styles.inputError : ''}
          />
          {errors.name && <span className={styles.errorText}>{errors.name}</span>}
        </div>

        {/* Email Field */}
        <div className={styles.formGroup}>
          <label htmlFor="email">Email *</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="your.email@example.com"
            disabled={isSubmitting}
            className={errors.email ? styles.inputError : ''}
          />
          {errors.email && <span className={styles.errorText}>{errors.email}</span>}
        </div>

        {/* Password Field */}
        <div className={styles.formGroup}>
          <label htmlFor="password">Password *</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="Minimum 8 characters"
            disabled={isSubmitting}
            className={errors.password ? styles.inputError : ''}
          />
          {errors.password && <span className={styles.errorText}>{errors.password}</span>}
        </div>

        {/* Background Questions */}
        <div className={styles.backgroundSection}>
          <h3>Tell us about your background</h3>
          <p className={styles.helpText}>
            This helps us personalize responses to match your expertise level
          </p>

          {/* Programming Experience */}
          <div className={styles.formGroup}>
            <label htmlFor="progExp">Programming Experience *</label>
            <select
              id="progExp"
              value={progExp}
              onChange={(e) => setProgExp(e.target.value)}
              disabled={isSubmitting}
              className={errors.progExp ? styles.inputError : ''}
            >
              <option value="">Select your experience</option>
              <option value="0-2 years">0-2 years (Beginner)</option>
              <option value="3-5 years">3-5 years (Intermediate)</option>
              <option value="6-10 years">6-10 years (Advanced)</option>
              <option value="10+ years">10+ years (Expert)</option>
            </select>
            {errors.progExp && <span className={styles.errorText}>{errors.progExp}</span>}
          </div>

          {/* ROS 2 Familiarity */}
          <div className={styles.formGroup}>
            <label htmlFor="ros2Fam">ROS 2 Familiarity *</label>
            <select
              id="ros2Fam"
              value={ros2Fam}
              onChange={(e) => setRos2Fam(e.target.value)}
              disabled={isSubmitting}
              className={errors.ros2Fam ? styles.inputError : ''}
            >
              <option value="">Select your familiarity</option>
              <option value="None">None (New to ROS 2)</option>
              <option value="Basic">Basic (Completed tutorials)</option>
              <option value="Intermediate">Intermediate (Built projects)</option>
              <option value="Advanced">Advanced (Professional experience)</option>
            </select>
            {errors.ros2Fam && <span className={styles.errorText}>{errors.ros2Fam}</span>}
          </div>

          {/* Hardware Access */}
          <div className={styles.formGroup}>
            <label htmlFor="hardware">Hardware Access *</label>
            <select
              id="hardware"
              value={hardware}
              onChange={(e) => setHardware(e.target.value)}
              disabled={isSubmitting}
              className={errors.hardware ? styles.inputError : ''}
            >
              <option value="">Select your access</option>
              <option value="None">None (Conceptual learning)</option>
              <option value="Simulation only">Simulation only (Gazebo/Isaac Sim)</option>
              <option value="Physical robots/sensors">Physical robots/sensors</option>
            </select>
            {errors.hardware && <span className={styles.errorText}>{errors.hardware}</span>}
          </div>
        </div>

        {/* Submit Error */}
        {errors.submit && (
          <div className={styles.submitError}>{errors.submit}</div>
        )}

        {/* Submit Button */}
        <button
          type="submit"
          className={styles.submitButton}
          disabled={isSubmitting}
        >
          {isSubmitting ? 'Creating Account...' : 'Sign Up'}
        </button>

        {/* Switch to Sign In */}
        <div className={styles.switchForm}>
          Already have an account?{' '}
          <button
            type="button"
            onClick={onSwitchToSignin}
            className={styles.linkButton}
            disabled={isSubmitting}
          >
            Sign In
          </button>
        </div>
      </form>
    </div>
  );
}
