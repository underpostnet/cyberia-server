package api

import (
	"context"
	"errors"
	"net/http"
	"strings"
	"time"

	"github.com/golang-jwt/jwt/v5"
)

// Roles in ascending privilege order.
const (
	RoleGuest     = "guest"
	RoleUser      = "user"
	RoleModerator = "moderator"
	RoleAdmin     = "admin"
)

var roleRank = map[string]int{
	RoleGuest:     0,
	RoleUser:      1,
	RoleModerator: 2,
	RoleAdmin:     3,
}

// Claims holds JWT claims including role.
type Claims struct {
	Sub  string `json:"sub"`
	Role string `json:"role"`
	jwt.RegisteredClaims
}

// GenerateToken creates a signed JWT.
func GenerateToken(secret, issuer, subject, role string, ttl time.Duration) (string, error) {
	claims := &Claims{
		Sub:  subject,
		Role: role,
		RegisteredClaims: jwt.RegisteredClaims{
			Issuer:    issuer,
			Subject:   subject,
			ExpiresAt: jwt.NewNumericDate(time.Now().Add(ttl)),
			IssuedAt:  jwt.NewNumericDate(time.Now()),
		},
	}
	t := jwt.NewWithClaims(jwt.SigningMethodHS256, claims)
	return t.SignedString([]byte(secret))
}

// ParseToken validates a JWT and returns Claims.
func ParseToken(secret, tokenStr string) (*Claims, error) {
	parser := jwt.NewParser(jwt.WithValidMethods([]string{jwt.SigningMethodHS256.Alg()}))
	claims := &Claims{}
	_, err := parser.ParseWithClaims(tokenStr, claims, func(t *jwt.Token) (interface{}, error) {
		return []byte(secret), nil
	})
	if err != nil {
		return nil, err
	}
	return claims, nil
}

// Context keys

type ctxKey string

const (
	ctxClaims ctxKey = "claims"
)

// AuthMiddleware authenticates JWT tokens in Authorization: Bearer header.
func AuthMiddleware(cfg Config) func(http.Handler) http.Handler {
	return func(next http.Handler) http.Handler {
		return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			authHeader := r.Header.Get("Authorization")
			if authHeader == "" {
				errorJSON(w, http.StatusUnauthorized, "missing Authorization header")
				return
			}
			parts := strings.SplitN(authHeader, " ", 2)
			if len(parts) != 2 || !strings.EqualFold(parts[0], "Bearer") {
				errorJSON(w, http.StatusUnauthorized, "invalid Authorization header format")
				return
			}
			claims, err := ParseToken(cfg.JWTSecret, parts[1])
			if err != nil {
				errorJSON(w, http.StatusUnauthorized, "invalid or expired token")
				return
			}
			ctx := context.WithValue(r.Context(), ctxClaims, claims)
			next.ServeHTTP(w, r.WithContext(ctx))
		})
	}
}

func getClaims(r *http.Request) (*Claims, error) {
	v := r.Context().Value(ctxClaims)
	if v == nil {
		return nil, errors.New("no claims in context")
	}
	c, ok := v.(*Claims)
	if !ok {
		return nil, errors.New("bad claims type")
	}
	return c, nil
}

// RequireRole ensures user has at least the required role.
func RequireRole(minRole string) func(http.Handler) http.Handler {
	return func(next http.Handler) http.Handler {
		return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			claims, err := getClaims(r)
			if err != nil {
				errorJSON(w, http.StatusUnauthorized, "unauthenticated")
				return
			}
			if roleRank[claims.Role] < roleRank[minRole] {
				errorJSON(w, http.StatusForbidden, "insufficient role")
				return
			}
			next.ServeHTTP(w, r)
		})
	}
}
