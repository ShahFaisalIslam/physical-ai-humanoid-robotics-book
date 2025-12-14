#!/bin/bash

# Install tools
npm install -g @anthropic-ai/claude-code @musistudio/claude-code-router

# Create config directories
mkdir -p ~/.claude-code-router ~/.claude

# Create router config
# Create new config with native Gemini endpoint
cat > ~/.claude-code-router/config.json << 'EOF'
{
  "LOG": true,
  "LOG_LEVEL": "info",
  "HOST": "127.0.0.1",
  "PORT": 3456,
  "API_TIMEOUT_MS": 600000,
  "Providers": [
    {
      "name": "gemini",
      "api_base_url": "https://generativelanguage.googleapis.com/v1beta/models/",
      "api_key": "$GOOGLE_API_KEY",
      "models": [
        "gemini-2.5-flash",
        "gemini-2.0-flash"
      ],
      "transformer": {
        "use": ["gemini"]
      }
    }
  ],
  "Router": {
    "default": "gemini,gemini-2.5-flash",
    "background": "gemini,gemini-2.5-flash",
    "think": "gemini,gemini-2.5-flash",
    "longContext": "gemini,gemini-2.5-flash",
    "longContextThreshold": 60000
  }
}
EOF

# Verify file was created
cat ~/.claude-code-router/config.json

# Set your API key (REPLACE "YOUR_KEY_HERE" with actual key!)
echo 'export GOOGLE_API_KEY="AIzaSyBq-upzrtQlOQywneBmDbNjOExzV0Bz_3M"' >> ~/.bashrc
source ~/.bashrc