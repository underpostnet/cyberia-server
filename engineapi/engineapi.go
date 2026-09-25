// Package engineapi names the versioned REST contract of engine-cyberia.
//
// The engine's DOMAIN_API_VERSION (src/server/domain/api-contract.js) is the one authority
// for the version; the engine's test suite checks that Base matches it.
package engineapi

// Base is the path every engine API lives under.
const Base = "/api/v1"

// Path joins one engine route to Base, e.g. Path("/cyberia-server-registry").
func Path(route string) string { return Base + route }
