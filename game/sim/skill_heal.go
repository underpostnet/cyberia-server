package sim

import (
	"time"

	"cyberia-server/game/content"
)

// executeZeroWasteHeal consumes one held unit of the trigger item once the
// caster's missing life takes its whole heal and the consume cooldown is over.
// The action cooldown rule times the next one.
func (s *Sim) executeZeroWasteHeal(ctx SkillContext) {
	ms, id := ctx.MapState, ctx.CasterID
	now := time.Now()
	heal := s.ConsumableHeal(ctx.TriggerItemID)
	vitals, mortal := ms.Vt(id), ms.Mt(id)
	if vitals.Ghost() || now.Before(mortal.NextConsumeAt) || heal <= 0 || vitals.MaxLife-vitals.Life < heal {
		return
	}
	if content.LayerQuantity(ms.Ap(id).Layers, ctx.TriggerItemID) < 1 {
		return
	}
	s.ConsumeItem(ms, id, ctx.TriggerItemID)
	content.RemoveLayerItem(&ms.Ap(id).Layers, ctx.TriggerItemID, 1)
	mortal.NextConsumeAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(id, ms)))
}
