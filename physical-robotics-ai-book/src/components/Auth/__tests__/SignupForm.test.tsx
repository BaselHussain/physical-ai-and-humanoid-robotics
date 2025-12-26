/**
 * Tests for SignupForm component
 *
 * Covers:
 * - Form rendering
 * - Input validation
 * - Background profile collection
 * - Successful submission
 * - Error handling
 * - Form switching
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import SignupForm from '../SignupForm';

describe('SignupForm', () => {
  const mockOnSubmit = jest.fn();
  const mockOnSwitchToSignin = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('Form rendering', () => {
    it('should render all form fields', () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      expect(screen.getByLabelText(/full name/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/email/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/password/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/programming experience/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/ros 2 familiarity/i)).toBeInTheDocument();
      expect(screen.getByLabelText(/hardware access/i)).toBeInTheDocument();
    });

    it('should render submit button', () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      expect(screen.getByRole('button', { name: /sign up/i })).toBeInTheDocument();
    });

    it('should render switch to signin button', () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      expect(screen.getByRole('button', { name: /sign in/i })).toBeInTheDocument();
    });

    it('should render background section with description', () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      expect(screen.getByText(/tell us about your background/i)).toBeInTheDocument();
      expect(
        screen.getByText(/this helps us personalize responses/i)
      ).toBeInTheDocument();
    });
  });

  describe('Form validation', () => {
    it('should show error when name is empty', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/name is required/i)).toBeInTheDocument();
      });

      expect(mockOnSubmit).not.toHaveBeenCalled();
    });

    it('should show error when name is too short', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const nameInput = screen.getByLabelText(/full name/i);
      await userEvent.type(nameInput, 'A');

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/name must be at least 2 characters/i)).toBeInTheDocument();
      });
    });

    it('should show error when email is empty', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/email is required/i)).toBeInTheDocument();
      });
    });

    it('should show error for invalid email format', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const emailInput = screen.getByLabelText(/email/i);
      await userEvent.type(emailInput, 'invalid-email');

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/invalid email format/i)).toBeInTheDocument();
      });
    });

    it('should show error when password is empty', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/password is required/i)).toBeInTheDocument();
      });
    });

    it('should show error when password is too short', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const passwordInput = screen.getByLabelText(/password/i);
      await userEvent.type(passwordInput, 'short');

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(
          screen.getByText(/password must be at least 8 characters/i)
        ).toBeInTheDocument();
      });
    });

    it('should show error when programming experience is not selected', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/programming experience is required/i)).toBeInTheDocument();
      });
    });

    it('should show error when ROS 2 familiarity is not selected', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/ros 2 familiarity is required/i)).toBeInTheDocument();
      });
    });

    it('should show error when hardware access is not selected', async () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/hardware access is required/i)).toBeInTheDocument();
      });
    });
  });

  describe('Successful submission', () => {
    it('should submit form with valid data', async () => {
      mockOnSubmit.mockResolvedValue(undefined);

      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      // Fill in form
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');

      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '3-5 years'
      );
      await userEvent.selectOptions(screen.getByLabelText(/ros 2 familiarity/i), 'Basic');
      await userEvent.selectOptions(
        screen.getByLabelText(/hardware access/i),
        'Simulation only'
      );

      // Submit
      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(mockOnSubmit).toHaveBeenCalledWith(
          'test@example.com',
          'password123',
          'Test User',
          {
            programming_experience: '3-5 years',
            ros2_familiarity: 'Basic',
            hardware_access: 'Simulation only',
          }
        );
      });
    });

    it('should show loading state during submission', async () => {
      let resolveSubmit: () => void;
      const submitPromise = new Promise<void>((resolve) => {
        resolveSubmit = resolve;
      });
      mockOnSubmit.mockReturnValue(submitPromise);

      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      // Fill in form
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');
      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '0-2 years'
      );
      await userEvent.selectOptions(screen.getByLabelText(/ros 2 familiarity/i), 'None');
      await userEvent.selectOptions(screen.getByLabelText(/hardware access/i), 'None');

      // Submit
      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      // Should show loading state
      await waitFor(() => {
        expect(screen.getByText(/creating account/i)).toBeInTheDocument();
      });

      // Resolve submission
      resolveSubmit!();

      await waitFor(() => {
        expect(screen.getByText(/sign up/i)).toBeInTheDocument();
      });
    });

    it('should disable form fields during submission', async () => {
      let resolveSubmit: () => void;
      const submitPromise = new Promise<void>((resolve) => {
        resolveSubmit = resolve;
      });
      mockOnSubmit.mockReturnValue(submitPromise);

      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      // Fill in form
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');
      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '10+ years'
      );
      await userEvent.selectOptions(
        screen.getByLabelText(/ros 2 familiarity/i),
        'Advanced'
      );
      await userEvent.selectOptions(
        screen.getByLabelText(/hardware access/i),
        'Physical robots/sensors'
      );

      // Submit
      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      // Check inputs are disabled
      await waitFor(() => {
        expect(screen.getByLabelText(/full name/i)).toBeDisabled();
        expect(screen.getByLabelText(/email/i)).toBeDisabled();
        expect(screen.getByLabelText(/password/i)).toBeDisabled();
      });

      // Resolve submission
      resolveSubmit!();
    });
  });

  describe('Error handling', () => {
    it('should display submission error', async () => {
      mockOnSubmit.mockRejectedValue(new Error('Email already exists'));

      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      // Fill in valid form
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');
      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '3-5 years'
      );
      await userEvent.selectOptions(screen.getByLabelText(/ros 2 familiarity/i), 'Basic');
      await userEvent.selectOptions(
        screen.getByLabelText(/hardware access/i),
        'Simulation only'
      );

      // Submit
      const submitButton = screen.getByRole('button', { name: /sign up/i });
      fireEvent.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText(/email already exists/i)).toBeInTheDocument();
      });
    });

    it('should clear errors on new submission attempt', async () => {
      mockOnSubmit.mockRejectedValueOnce(new Error('Error 1'));
      mockOnSubmit.mockResolvedValueOnce(undefined);

      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      // Fill and submit (will fail)
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');
      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '0-2 years'
      );
      await userEvent.selectOptions(screen.getByLabelText(/ros 2 familiarity/i), 'None');
      await userEvent.selectOptions(screen.getByLabelText(/hardware access/i), 'None');

      fireEvent.click(screen.getByRole('button', { name: /sign up/i }));

      await waitFor(() => {
        expect(screen.getByText(/error 1/i)).toBeInTheDocument();
      });

      // Submit again (will succeed)
      fireEvent.click(screen.getByRole('button', { name: /sign up/i }));

      await waitFor(() => {
        expect(screen.queryByText(/error 1/i)).not.toBeInTheDocument();
      });
    });
  });

  describe('Form switching', () => {
    it('should call onSwitchToSignin when clicking signin button', () => {
      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      const signinButton = screen.getByRole('button', { name: /sign in/i });
      fireEvent.click(signinButton);

      expect(mockOnSwitchToSignin).toHaveBeenCalledTimes(1);
    });

    it('should not switch forms during submission', async () => {
      let resolveSubmit: () => void;
      const submitPromise = new Promise<void>((resolve) => {
        resolveSubmit = resolve;
      });
      mockOnSubmit.mockReturnValue(submitPromise);

      render(<SignupForm onSubmit={mockOnSubmit} onSwitchToSignin={mockOnSwitchToSignin} />);

      // Fill form and submit
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');
      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '0-2 years'
      );
      await userEvent.selectOptions(screen.getByLabelText(/ros 2 familiarity/i), 'None');
      await userEvent.selectOptions(screen.getByLabelText(/hardware access/i), 'None');

      fireEvent.click(screen.getByRole('button', { name: /sign up/i }));

      // Try to click signin during submission
      await waitFor(() => {
        const signinButton = screen.getByRole('button', { name: /sign in/i });
        expect(signinButton).toBeDisabled();
      });

      // Resolve submission
      resolveSubmit!();
    });
  });
});
