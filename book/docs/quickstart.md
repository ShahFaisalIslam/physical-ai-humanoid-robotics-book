---
title: Quickstart Guide
description: Get started with the Physical AI & Humanoid Robotics book in minutes.
sidebar_position: 2
---

# Quickstart Guide

This guide will help you set up your local development environment to contribute to the Physical AI & Humanoid Robotics book.

## Prerequisites

Before getting started, make sure you have the following installed on your system:

- **Node.js**: Version 18.0 or higher
- **npm** or **yarn**: Package manager for Node.js
- **Git**: Version control system
- **GitHub Account**: For submitting changes via Pull Requests

## Installation Steps

1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-org/physical-ai-humanoid-robotics.git
   cd physical-ai-humanoid-robotics/book
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm start
   ```

4. **Open your browser:**
   Navigate to [http://localhost:3000](http://localhost:3000) to view the book locally.

## Making Changes

1. Edit any Markdown file in the `docs/` directory.
2. The site will automatically reload with your changes.
3. When you're ready to submit your changes, follow the contribution workflow outlined in the [Content Contribution Guide](./contributing).

## Common Commands

| Command | Description |
|--------|-------------|
| `npm start` | Starts the development server |
| `npm run build` | Builds the static website |
| `npm run serve` | Serves the built website locally |

## Need Help?

- Check out the [Preface](./preface) for an overview of the book
- Read the [Content Contribution Guide](./contributing) for detailed instructions
- Create an issue in the GitHub repository if you encounter problems