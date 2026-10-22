// Package content — market_profile.go
//
// A market profile is the shared contract of a market as this world sees it:
// the modes and durations it supports, the auction rules, the fees, the price
// range, and the scope key of this world. The engine delivers the profiles the
// terminals of the world name with the world, and the engine is the authority:
// a listing whose terms no longer match the stored profile is refused there.
// A profile with a defect opens no market here; a retired one serves its
// listings and claims and takes no new listing.
//
// Caller MUST hold s.mu.
package content

import (
	"slices"

	pb "cyberia-server/gen/proto"
)

// MarketProfile is the shared contract of one market in this world.
type MarketProfile struct {
	Code              string   `json:"code"`
	Revision          uint64   `json:"revision"`
	Status            string   `json:"status"`
	Scope             string   `json:"scope"`
	ScopeKey          string   `json:"scopeKey"`
	Ref               string   `json:"ref"`
	CurrencyCode      string   `json:"currencyCode"`
	Modes             []string `json:"modes"`
	Hours             []int    `json:"hours"`
	RaisePercent      int      `json:"raisePercent"`
	MinRaise          int      `json:"minRaise"`
	ListingFee        int      `json:"listingFee"`
	SaleFee           int      `json:"saleFee"`
	PriceMin          int      `json:"priceMin"`
	PriceMax          int      `json:"priceMax"`
	ListingsPerPlayer int      `json:"listingsPerPlayer"`
	// Defect is why the market opens nowhere in this world; empty when it opens.
	Defect string `json:"defect"`
}

// marketStatusActive is the status of a market that takes new listings.
const marketStatusActive = "active"

// invalidMarketProfile is the defect of a profile outside the engine contract.
const invalidMarketProfile = "the market profile breaks the engine contract"

func marketProfileFromProto(p *pb.MarketProfile) MarketProfile {
	profile := MarketProfile{
		Code: p.GetCode(), Revision: p.GetRevision(), Status: p.GetStatus(), Scope: p.GetScope(),
		ScopeKey: p.GetScopeKey(), Ref: p.GetRef(), CurrencyCode: p.GetCurrencyCode(), Modes: p.GetModes(),
		RaisePercent: int(p.GetRaisePercent()), MinRaise: int(p.GetMinRaise()),
		ListingFee: int(p.GetListingFee()), SaleFee: int(p.GetSaleFee()),
		PriceMin: int(p.GetPriceMin()), PriceMax: int(p.GetPriceMax()),
		ListingsPerPlayer: int(p.GetListingsPerPlayer()), Defect: p.GetDefect(),
	}
	for _, hours := range p.GetHours() {
		profile.Hours = append(profile.Hours, int(hours))
	}
	return profile
}

// valid reports whether the terms fit the engine contract: bounded rules,
// known modes and durations, a price range that is not empty, and a sale fee
// within the lowest price.
func (m MarketProfile) valid() bool {
	known := func(values []string, options []string) bool {
		return len(values) > 0 && !slices.ContainsFunc(values, func(v string) bool { return !slices.Contains(options, v) })
	}
	hours := len(m.Hours) > 0 && !slices.ContainsFunc(m.Hours, func(h int) bool { return !slices.Contains(MarketHourOptions, h) })
	return m.validBounds() && known(m.Modes, MarketModes) && hours && m.PriceMin <= m.PriceMax && m.SaleFee <= m.PriceMin
}

// listingModes are the modes a seller lists in at a terminal: the modes the
// market supports that the terminal offers.
func (m MarketProfile) listingModes(terminal ActionMarket) []string {
	return slices.DeleteFunc(slices.Clone(m.Modes), func(mode string) bool {
		return !slices.Contains(terminal.ListingModes, mode)
	})
}

// BindMarkets caches the market profiles the engine delivered with the world.
// A profile that breaks the engine contract opens no market.
//
// Caller MUST hold s.mu.
func (s *Content) BindMarkets(markets []*pb.MarketProfile) {
	s.marketProfiles = make(map[string]MarketProfile, len(markets))
	for _, p := range markets {
		profile := marketProfileFromProto(p)
		if profile.Defect == "" && !profile.valid() {
			profile.Defect = invalidMarketProfile
		}
		s.marketProfiles[profile.Code] = profile
	}
}
