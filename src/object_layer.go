package game

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"time"
)

// Stats describes the attribute distribution for an object layer.
type Stats struct {
	Effect       int `json:"effect"`
	Resistance   int `json:"resistance"`
	Agility      int `json:"agility"`
	Range        int `json:"range"`
	Intelligence int `json:"intelligence"`
	Utility      int `json:"utility"`
}

// Item describes a generic item.
type Item struct {
	ID          string `json:"id"`
	Type        string `json:"type"`
	Description string `json:"description"`
	Activable   bool   `json:"activable"`
}

// Ledger holds blockchain protocol metadata linking the visual object-layer
// prefab to its economic reality (token standard + smart-contract address).
type Ledger struct {
	Type    string `json:"type"`              // ERC20, ERC721, OFF_CHAIN
	Address string `json:"address,omitempty"` // Solidity contract address
}

// Render holds IPFS content identifiers for the consolidated atlas sprite sheet.
type Render struct {
	Cid         string `json:"cid,omitempty"`         // IPFS CID for the atlas PNG
	MetadataCid string `json:"metadataCid,omitempty"` // IPFS CID for the atlas metadata JSON
}

// ObjectLayerData groups the data for an ObjectLayer.
type ObjectLayerData struct {
	Stats  Stats   `json:"stats"`
	Item   Item    `json:"item"`
	Ledger *Ledger `json:"ledger,omitempty"`
	Render *Render `json:"render,omitempty"`
}

// ObjectLayer is the top level schema for an object layer.
// Render frames are now stored in a separate ObjectLayerRenderFrames collection,
// referenced by ObjectLayerRenderFramesId.
type ObjectLayer struct {
	ID                        string          `json:"_id,omitempty"`
	Data                      ObjectLayerData `json:"data"`
	Cid                       string          `json:"cid,omitempty"`
	ObjectLayerRenderFramesId interface{}     `json:"objectLayerRenderFramesId,omitempty"`
	AtlasSpriteSheetId        interface{}     `json:"atlasSpriteSheetId,omitempty"`
	Sha256                    string          `json:"sha256"`
	CreatedAt                 time.Time       `json:"createdAt,omitempty"`
	UpdatedAt                 time.Time       `json:"updatedAt,omitempty"`
}

// ---------- Cyberia API client ----------

const (
	defaultCyberiaAPIBaseURL = "https://www.cyberiaonline.com"
	apiAuthPath              = "/api/user/auth"
	apiObjectLayerPath       = "/api/object-layer"
	apiObjectLayerMetadata   = "/api/object-layer/metadata"
	httpTimeoutSeconds       = 30
	paginationLimit          = 100
)

// cyberiaAPIBaseURL returns the base URL from env or the default.
func cyberiaAPIBaseURL() string {
	if u := os.Getenv("CYBERIA_API_BASE_URL"); u != "" {
		return u
	}
	return defaultCyberiaAPIBaseURL
}

// apiResponse is the generic envelope returned by the Cyberia API.
type apiResponse struct {
	Status  string          `json:"status"`
	Message string          `json:"message,omitempty"`
	Data    json.RawMessage `json:"data"`
}

// authResponseData is the data payload from POST /api/user/auth.
type authResponseData struct {
	Token string          `json:"token"`
	User  json.RawMessage `json:"user"`
}

// objectLayerListData is the paginated data payload from GET /api/object-layer/.
type objectLayerListData struct {
	Data       []ObjectLayer `json:"data"`
	Total      int           `json:"total"`
	Page       int           `json:"page"`
	TotalPages int           `json:"totalPages"`
}

// CyberiaAPIAuthenticate authenticates against the Cyberia API and returns a JWT bearer token.
func CyberiaAPIAuthenticate(email, password string) (string, error) {
	baseURL := cyberiaAPIBaseURL()
	url := baseURL + apiAuthPath

	body, err := json.Marshal(map[string]string{
		"email":    email,
		"password": password,
	})
	if err != nil {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: marshal body: %w", err)
	}

	client := &http.Client{Timeout: httpTimeoutSeconds * time.Second}
	req, err := http.NewRequest("POST", url, bytes.NewReader(body))
	if err != nil {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: new request: %w", err)
	}
	req.Header.Set("Content-Type", "application/json")

	resp, err := client.Do(req)
	if err != nil {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: do request: %w", err)
	}
	defer resp.Body.Close()

	respBody, err := io.ReadAll(resp.Body)
	if err != nil {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: read body: %w", err)
	}

	if resp.StatusCode != http.StatusOK {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: unexpected status %d: %s", resp.StatusCode, string(respBody))
	}

	var envelope apiResponse
	if err := json.Unmarshal(respBody, &envelope); err != nil {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: unmarshal envelope: %w", err)
	}
	if envelope.Status != "success" {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: API error: %s", envelope.Message)
	}

	var authData authResponseData
	if err := json.Unmarshal(envelope.Data, &authData); err != nil {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: unmarshal auth data: %w", err)
	}
	if authData.Token == "" {
		return "", fmt.Errorf("CyberiaAPIAuthenticate: empty token in response")
	}

	return authData.Token, nil
}

// cyberiaAPIGet performs an authenticated GET request to the Cyberia API.
func cyberiaAPIGet(path, token string) ([]byte, error) {
	baseURL := cyberiaAPIBaseURL()
	url := baseURL + path

	client := &http.Client{Timeout: httpTimeoutSeconds * time.Second}
	req, err := http.NewRequest("GET", url, nil)
	if err != nil {
		return nil, fmt.Errorf("cyberiaAPIGet: new request: %w", err)
	}
	req.Header.Set("Authorization", "Bearer "+token)
	req.Header.Set("Content-Type", "application/json")

	resp, err := client.Do(req)
	if err != nil {
		return nil, fmt.Errorf("cyberiaAPIGet: do request: %w", err)
	}
	defer resp.Body.Close()

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("cyberiaAPIGet: read body: %w", err)
	}

	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("cyberiaAPIGet: unexpected status %d: %s", resp.StatusCode, string(body))
	}

	return body, nil
}

// FetchObjectLayerIDs fetches all object layer IDs from the paginated list endpoint.
func FetchObjectLayerIDs(token string) ([]string, error) {
	var allIDs []string
	page := 1

	for {
		path := fmt.Sprintf("%s/?page=%d&limit=%d", apiObjectLayerPath, page, paginationLimit)
		body, err := cyberiaAPIGet(path, token)
		if err != nil {
			return nil, fmt.Errorf("FetchObjectLayerIDs page %d: %w", page, err)
		}

		var envelope apiResponse
		if err := json.Unmarshal(body, &envelope); err != nil {
			return nil, fmt.Errorf("FetchObjectLayerIDs: unmarshal envelope page %d: %w", page, err)
		}
		if envelope.Status != "success" {
			return nil, fmt.Errorf("FetchObjectLayerIDs: API error page %d: %s", page, envelope.Message)
		}

		var listData objectLayerListData
		if err := json.Unmarshal(envelope.Data, &listData); err != nil {
			return nil, fmt.Errorf("FetchObjectLayerIDs: unmarshal list data page %d: %w", page, err)
		}

		for _, ol := range listData.Data {
			if ol.ID != "" {
				allIDs = append(allIDs, ol.ID)
			}
		}

		log.Printf("FetchObjectLayerIDs: page %d/%d, fetched %d items (total: %d)",
			page, listData.TotalPages, len(listData.Data), listData.Total)

		if page >= listData.TotalPages || len(listData.Data) == 0 {
			break
		}
		page++
	}

	return allIDs, nil
}

// FetchObjectLayerMetadata fetches the full metadata for a single object layer by ID.
func FetchObjectLayerMetadata(id, token string) (*ObjectLayer, error) {
	path := fmt.Sprintf("%s/%s", apiObjectLayerMetadata, id)
	body, err := cyberiaAPIGet(path, token)
	if err != nil {
		return nil, fmt.Errorf("FetchObjectLayerMetadata(%s): %w", id, err)
	}

	var envelope apiResponse
	if err := json.Unmarshal(body, &envelope); err != nil {
		return nil, fmt.Errorf("FetchObjectLayerMetadata(%s): unmarshal envelope: %w", id, err)
	}
	if envelope.Status != "success" {
		return nil, fmt.Errorf("FetchObjectLayerMetadata(%s): API error: %s", id, envelope.Message)
	}

	var ol ObjectLayer
	if err := json.Unmarshal(envelope.Data, &ol); err != nil {
		return nil, fmt.Errorf("FetchObjectLayerMetadata(%s): unmarshal object layer: %w", id, err)
	}

	return &ol, nil
}

// FetchAllObjectLayers authenticates and fetches all object layers from the Cyberia API,
// returning a cache map keyed by data.item.id.
func FetchAllObjectLayers() (map[string]*ObjectLayer, error) {
	email := os.Getenv("CYBERIA_API_EMAIL")
	password := os.Getenv("CYBERIA_API_PASSWORD")
	if email == "" || password == "" {
		return nil, fmt.Errorf("FetchAllObjectLayers: CYBERIA_API_EMAIL and CYBERIA_API_PASSWORD env vars must be set")
	}

	log.Printf("Authenticating with Cyberia API at %s ...", cyberiaAPIBaseURL())
	token, err := CyberiaAPIAuthenticate(email, password)
	if err != nil {
		return nil, fmt.Errorf("FetchAllObjectLayers: authentication failed: %w", err)
	}
	log.Println("Cyberia API authentication successful.")

	// Step 1: Fetch all object layer IDs from the list endpoint
	ids, err := FetchObjectLayerIDs(token)
	if err != nil {
		return nil, fmt.Errorf("FetchAllObjectLayers: fetch IDs: %w", err)
	}
	log.Printf("Found %d object layers to fetch metadata for.", len(ids))

	// Step 2: Fetch full metadata for each object layer
	cache := make(map[string]*ObjectLayer, len(ids))
	fetchErrors := 0

	for i, id := range ids {
		ol, err := FetchObjectLayerMetadata(id, token)
		if err != nil {
			log.Printf("Warning: failed to fetch metadata for object layer %s: %v", id, err)
			fetchErrors++
			continue
		}

		itemID := ol.Data.Item.ID
		if itemID == "" {
			log.Printf("Warning: object layer %s has empty item ID, skipping.", id)
			fetchErrors++
			continue
		}

		cache[itemID] = ol

		if (i+1)%25 == 0 || i+1 == len(ids) {
			log.Printf("Fetched metadata for %d/%d object layers (%d errors).", i+1, len(ids), fetchErrors)
		}
	}

	log.Printf("Object layer cache built: %d items cached, %d errors.", len(cache), fetchErrors)
	return cache, nil
}
