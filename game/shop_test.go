package game

import (
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// The vendor catalog crosses a process boundary (engine-cyberia → gRPC → here),
// so the decode and the row/price selection the purchase handler depends on are
// pinned here rather than only exercised through a live world build.

func TestProtoToActionDecodesShopItems(t *testing.T) {
	msg := &pb.CyberiaActionMessage{
		Code:          "loc-fallback-map-0-18-16",
		Label:         "Punk",
		SourceMapCode: "fallback-map-0",
		SourceCellX:   18,
		SourceCellY:   16,
		DialogCode:    "default-punk",
		ShopItems: []*pb.ActionShopItem{
			{ItemId: "tim-knife", PriceItemId: "coin", PriceQty: 10},
		},
	}

	action := protoToAction(msg)
	if len(action.ShopItems) != 1 {
		t.Fatalf("want 1 shop item, got %d", len(action.ShopItems))
	}
	got := action.ShopItems[0]
	if got.ItemID != "tim-knife" || got.PriceItemID != "coin" || got.PriceQty != 10 {
		t.Fatalf("shop item did not survive the wire: %+v", got)
	}
}

func TestShopItemForSelectsByItemID(t *testing.T) {
	action := &CyberiaAction{ShopItems: []ActionShopItem{
		{ItemID: "tim-knife", PriceItemID: "coin", PriceQty: 10},
		{ItemID: "hatchet", PriceItemID: "coin", PriceQty: 25},
	}}

	if item := shopItemFor(action, "hatchet"); item == nil || item.PriceQty != 25 {
		t.Fatalf("want the hatchet row, got %+v", item)
	}
	if item := shopItemFor(action, "wood-drop-1"); item != nil {
		t.Fatalf("want no row for an item not on sale, got %+v", item)
	}
	if item := shopItemFor(nil, "tim-knife"); item != nil {
		t.Fatal("want no row for an entity with no bound action")
	}
	if item := shopItemFor(&CyberiaAction{}, ""); item != nil {
		t.Fatal("want no row for an empty item id")
	}
}

func TestShopPriceFillsSchemaDefaults(t *testing.T) {
	s := &Content{World: &World{}, coinItemID: "coin"}

	// An unset currency falls back to coins, matching the CyberiaAction schema.
	itemID, qty := s.shopPrice(&ActionShopItem{ItemID: "tim-knife", PriceQty: 10})
	if itemID != "coin" || qty != 10 {
		t.Fatalf("want coin/10, got %s/%d", itemID, qty)
	}
	// A negative quantity can never credit the buyer.
	if _, qty := s.shopPrice(&ActionShopItem{ItemID: "tim-knife", PriceQty: -5}); qty != 0 {
		t.Fatalf("want a negative price clamped to 0, got %d", qty)
	}
	itemID, qty = s.shopPrice(&ActionShopItem{ItemID: "hatchet", PriceItemID: "wood-drop-1", PriceQty: 3})
	if itemID != "wood-drop-1" || qty != 3 {
		t.Fatalf("want wood-drop-1/3, got %s/%d", itemID, qty)
	}
}

func TestShopBuyQuantityClampsToRange(t *testing.T) {
	// A client that omits the quantity (0) or an older one that predates the
	// field means one unit; nothing may exceed the authoritative cap.
	cases := []struct{ requested, want int }{
		{0, 1}, {-3, 1}, {1, 1}, {shopBuyMaxQty, shopBuyMaxQty},
		{shopBuyMaxQty + 1, shopBuyMaxQty}, {255, shopBuyMaxQty},
	}
	for _, c := range cases {
		if got := shopBuyQuantity(c.requested); got != c.want {
			t.Fatalf("shopBuyQuantity(%d) = %d, want %d", c.requested, got, c.want)
		}
	}
}

// botHasShop drives the action-provider capability bit, so a vendor advertises
// itself overhead exactly like a pending action-talk does.
func TestBotHasShopGatesTheCapabilityBit(t *testing.T) {
	catalog := &CyberiaAction{ShopItems: []ActionShopItem{{ItemID: "tim-knife", PriceQty: 10}}}
	s := &Content{World: &World{}, actionCache: map[EntityID]*CyberiaAction{
		1: catalog,
		2: {DialogCode: "default-lain"},
		3: catalog,
	}}
	vendor := placeBot(s.World, 1, Vitals{})
	plain := placeBot(s.World, 2, Vitals{})
	dead := placeBot(s.World, 3, Vitals{RespawnAt: time.Now().Add(time.Second)})

	if !s.botHasShop(vendor, testMap(s.World)) {
		t.Fatal("a live bot with a catalog is a vendor")
	}
	if s.botHasShop(plain, testMap(s.World)) {
		t.Fatal("an action with no catalog is not a vendor")
	}
	if s.botHasShop(dead, testMap(s.World)) {
		t.Fatal("a dead vendor must not advertise")
	}
	if s.botHasShop(placeBot(s.World, 4, Vitals{}), testMap(s.World)) {
		t.Fatal("a bot with no bound action is not a vendor")
	}
	if s.botHasShop(5, nil) {
		t.Fatal("nil bot is not a vendor")
	}

	if flags := s.botInteractionFlags(false, false, true); flags&InteractionFlagAction == 0 {
		t.Fatalf("a vendor must light the action bit, got 0x%x", flags)
	}
	if flags := s.botInteractionFlags(false, false, false); flags != 0 {
		t.Fatalf("a plain bot must light nothing, got 0x%x", flags)
	}
}
