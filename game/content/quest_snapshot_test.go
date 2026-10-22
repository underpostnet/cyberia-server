package content

import (
	"slices"
	"testing"
)

// The snapshot names the step a quest is on and counts each of its objectives,
// so the client resolves them without reading text. It is a copy of that step.
func TestAQuestSnapshotNamesItsStepAndCountsItsObjectives(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(4))
	s.questDefs = map[string]*CyberiaQuest{"q": {Code: "q", Steps: []QuestStep{
		{ID: "hunt", Description: "Hunt the rats", Objectives: []QuestObjective{
			{Type: "kill", ItemID: "rat", Quantity: 2},
			{Type: "talk", ItemID: "sage", Quantity: 1},
		}},
		{ID: "report", Objectives: []QuestObjective{{Type: "collect", ItemID: "tail", Quantity: 3}}},
	}}}
	s.grantQuest(player, "q")
	s.AdvancePlayerQuestsOnKill(player, "rat")
	qp, _ := s.playerQuest(player, "q")
	snapshot := s.questSnapshot(qp)
	want := []QuestObjectiveProgress{
		{Type: "kill", ItemID: "rat", Current: 1, Required: 2},
		{Type: "talk", ItemID: "sage", Current: 0, Required: 1},
	}
	if snapshot.StepID != "hunt" || snapshot.ActiveStep != "Hunt the rats" || !slices.Equal(snapshot.Objectives, want) {
		t.Fatalf("snapshot = %+v", snapshot)
	}
	s.AdvancePlayerQuestsOnKill(player, "rat")
	if snapshot.Objectives[0].Current != 1 {
		t.Fatal("a snapshot keeps the counts it was taken with")
	}
}
