package api

import (
	"errors"
	"fmt"
	"unicode"
)

// Password complexity requirements
const (
	MinPasswordLength = 8
	MaxPasswordLength = 72 // bcrypt limit is 72 bytes
)

var (
	ErrPasswordTooShort       = fmt.Errorf("password must be at least %d characters long", MinPasswordLength)
	ErrPasswordTooLong        = fmt.Errorf("password must be at most %d characters long", MaxPasswordLength)
	ErrPasswordNoUppercase    = errors.New("password must contain at least one uppercase letter")
	ErrPasswordNoLowercase    = errors.New("password must contain at least one lowercase letter")
	ErrPasswordNoDigit        = errors.New("password must contain at least one digit")
	ErrPasswordNoSpecial      = errors.New("password must contain at least one special character")
	ErrPasswordContainsSpaces = errors.New("password must not contain spaces")
)

// ValidatePassword checks if the password meets industry standard complexity rules.
//
// Rules:
// - At least 8 characters long
// - At most 72 characters long (bcrypt limit)
// - At least one uppercase letter
// - At least one lowercase letter
// - At least one digit
// - At least one special character (punctuation or symbol)
// - No whitespace
//
// Examples of valid passwords:
// - "CorrectHorseBatteryStaple1!"
// - "Tr0ub4dor&3"
// - "S3cur3P@ssw0rd!"
// - "Xy9#mK2$pL"
func ValidatePassword(password string) error {
	if len(password) < MinPasswordLength {
		return ErrPasswordTooShort
	}
	if len(password) > MaxPasswordLength {
		return ErrPasswordTooLong
	}

	var (
		hasUpper   bool
		hasLower   bool
		hasDigit   bool
		hasSpecial bool
	)

	for _, char := range password {
		switch {
		case unicode.IsSpace(char):
			return ErrPasswordContainsSpaces
		case unicode.IsUpper(char):
			hasUpper = true
		case unicode.IsLower(char):
			hasLower = true
		case unicode.IsDigit(char):
			hasDigit = true
		case unicode.IsPunct(char) || unicode.IsSymbol(char):
			hasSpecial = true
		}
	}

	if !hasUpper {
		return ErrPasswordNoUppercase
	}
	if !hasLower {
		return ErrPasswordNoLowercase
	}
	if !hasDigit {
		return ErrPasswordNoDigit
	}
	if !hasSpecial {
		return ErrPasswordNoSpecial
	}

	return nil
}
