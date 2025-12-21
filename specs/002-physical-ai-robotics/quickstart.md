# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Overview
This guide helps you quickly set up and start contributing to the Physical AI & Humanoid Robotics book project. The book is built with Docusaurus and published to GitHub Pages, requiring no local ROS 2 installation for development.

## Prerequisites

Before getting started, ensure your system meets these requirements:

### System Requirements
- **Operating System**: Any modern OS (Linux, macOS, Windows with WSL)
- **Node.js**: Version 18.0 or higher
- **npm or yarn**: Package manager for Node.js
- **Git**: Version control system
- **Disk Space**: 500MB free space for dependencies

### Software Requirements
- **Node.js**: Download from [nodejs.org](https://nodejs.org/)
- **Git**: Download from [git-scm.com](https://git-scm.com/)

## Installation Steps

### Step 1: Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book
```

### Step 2: Install Dependencies
```bash
cd book
npm install
```

### Step 3: Start Local Development Server
```bash
npm start
```

This command starts a local development server and opens the book in your browser. Most changes are reflected live without restarting the server.

## Project Structure

The book follows a modular structure organized by the four main modules:

```
book/
├── docs/
│   ├── module-1-ros-fundamentals/     # ROS 2 basics
│   ├── module-2-digital-twin/        # Simulation with Gazebo & Unity
│   ├── module-3-ai-robot-brain/      # NVIDIA Isaac platform
│   ├── module-4-vision-language-action/ # VLA and conversational robotics
│   └── capstone-project/             # Complete project integrating all concepts
├── src/
│   ├── components/                   # Custom React components
│   └── theme/                        # Custom theme components
├── static/                          # Static assets (images, videos)
├── docusaurus.config.js             # Main Docusaurus configuration
├── sidebars.js                      # Navigation sidebar configuration
└── package.json                     # Project dependencies and scripts
```

## Contributing Content

### Adding a New Page
1. Create a new Markdown file in the appropriate module directory
2. Add the page to the `sidebars.js` file to make it appear in the navigation
3. Use the frontmatter to specify metadata:

```markdown
---
title: Page Title
description: Brief description of the page content
sidebar_position: 3
---

# Page Title

Content goes here...
```

### Adding Code Examples
Use Docusaurus' built-in code block syntax with syntax highlighting:

```markdown
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="python" label="Python" default>

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
```

</TabItem>
<TabItem value="bash" label="Bash">

```bash
ros2 run demo_nodes_cpp talker
```

</TabItem>
</Tabs>
```

### Adding Diagrams
Place image files in the `static/img/` directory and reference them:

```markdown
![Diagram Description](/img/diagram-name.png)
```

## Building and Deploying

### Build for Production
```bash
npm run build
```

This command generates static content in the `build/` directory, which can be served using any static hosting service.

### Deploy to GitHub Pages
The repository is configured with GitHub Actions to automatically deploy to GitHub Pages when changes are pushed to the main branch. No manual deployment steps are required.

## Writing Guidelines

### Content Structure
- Each module should have 3-5 chapters covering specific topics
- Start chapters with learning objectives
- End chapters with a summary and key takeaways
- Include code examples with explanations
- Use diagrams to illustrate complex concepts

### Code Examples
- Provide context for each code example
- Include comments explaining key parts
- Show expected outputs where applicable
- Follow ROS 2 best practices and conventions

### Accessibility
- Use proper heading hierarchy (H1, H2, H3, etc.)
- Provide alt text for all images
- Use descriptive link text
- Ensure sufficient color contrast

## Troubleshooting

### Common Issues

#### Issue: "Command not found: npm"
**Solution**: Install Node.js from [nodejs.org](https://nodejs.org/), which includes npm.

#### Issue: Local server doesn't start
**Solution**: Make sure you're running the command from the `book/` directory:
```bash
cd book
npm start
```

#### Issue: Dependencies not installing
**Solution**: Clear npm cache and try again:
```bash
npm cache clean --force
rm -rf node_modules package-lock.json
npm install
```

## Next Steps

1. Review the existing content in the `docs/` directory
2. Identify areas where you can contribute new content
3. Create a new branch for your changes
4. Add your content following the guidelines above
5. Test your changes locally with `npm start`
6. Submit a pull request for review

## Support

If you encounter issues not covered in this guide:
- Check the Docusaurus documentation at [docusaurus.io](https://docusaurus.io/)
- Submit an issue on the book's GitHub repository
- Join our community forum for additional support

---

**Congratulations!** You now have a fully functional local development environment for the Physical AI & Humanoid Robotics book. Start exploring the existing content and contribute to this comprehensive educational resource.