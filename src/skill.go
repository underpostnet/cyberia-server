package game

import (
	"log"
)

// SkillDefinition defines the properties of a skill, including the logic to execute
// and any associated item IDs (e.g., for spawned entities like bullets).
type SkillDefinition struct {
	ItemIDs      []string
	LogicEventID string
}

// SkillConfig maps a triggering ItemID to a list of SkillDefinitions.
// This provides a declarative way to associate items with behaviors.
var SkillConfig = map[string][]SkillDefinition{
	"anon":             {{LogicEventID: "doppelganger"}},
	"atlas_pistol_mk2": {{ItemIDs: []string{"atlas_pistol_mk2_bullet"}, LogicEventID: "atlas_pistol_mk2_logic"}},
	"coin":             {{LogicEventID: "coin_drop_or_transaction"}},
	"purple":           {{LogicEventID: "doppelganger"}},
}

// HandlePlayerActionSkills checks if a player's action triggers any skills
// based on their active items and calls the corresponding skill logic.
func (s *GameServer) HandlePlayerActionSkills(player *PlayerState, mapState *MapState, target Point) {
	for _, layer := range player.ObjectLayers {
		if !layer.Active {
			continue
		}

		// Check if the active item has any associated skills in our config
		if skillDefs, ok := SkillConfig[layer.ItemID]; ok {
			for _, skillDef := range skillDefs {
				// Execute the skill logic based on its ID
				switch skillDef.LogicEventID {
				case "doppelganger":
					s.executePlayerDoppelgangerSkill(player, mapState, layer.ItemID)
				case "atlas_pistol_mk2_logic":
					s.executePlayerBulletSkill(player, mapState, skillDef, target)
				// Future skills like "teleport_burst" or "invisibility" could be added here.
				default:
					log.Printf("Unknown LogicEventID '%s' for item '%s'", skillDef.LogicEventID, layer.ItemID)
				}
			}
		}
	}
}

// handleBotSkills checks and executes skills for a bot when it takes an "action" (like finding a new path).
func (s *GameServer) handleBotSkills(bot *BotState, mapState *MapState, target Point) {
	for _, layer := range bot.ObjectLayers {
		if !layer.Active {
			continue
		}

		// Check if the active item has any associated skills in our config
		if skillDefs, ok := SkillConfig[layer.ItemID]; ok {
			for _, skillDef := range skillDefs {
				// Execute the skill logic based on its ID
				switch skillDef.LogicEventID {
				case "doppelganger":
					s.executeBotDoppelgangerSkill(bot, mapState, layer.ItemID)
				case "atlas_pistol_mk2_logic":
					s.executeBotBulletSkill(bot, mapState, skillDef, target)
				// Future skills could be added here.
				default:
					log.Printf("Unknown LogicEventID '%s' for item '%s' on bot %s", skillDef.LogicEventID, layer.ItemID, bot.ID)
				}
			}
		}
	}
}

// HandleOnKillSkills checks for skills that trigger when an entity is killed by a bot

func (s *GameServer) HandleOnKillSkills(killerBot *BotState, victim interface{}, mapState *MapState) { // The killer is a bullet bot. The actual killer is the caster of the bullet.
	// The killer is a bullet bot. The actual killer is the caster of the bullet.
	if killerBot.CasterID == "" {
		return
	}

	var caster interface{}

	// Find the caster, which can be a player or a bot.
	if p, ok := mapState.players[killerBot.CasterID]; ok {
		caster = p
	} else if b, ok := mapState.bots[killerBot.CasterID]; ok {
		caster = b
	} else {
		return // Caster not found in the map.
	}

	// --- Core Mechanic: on Kill Events ---
	// This is not a skill, but a fundamental rule: the victor loots the vanquished.
	s.executeCoinDropOnKill(caster, victim, s)

}
