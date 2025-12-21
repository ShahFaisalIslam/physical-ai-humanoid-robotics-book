---
title: Content Contribution Guide
description: A comprehensive guide for contributing content to the Physical AI & Humanoid Robotics book.
sidebar_position: 1
---

# Content Contribution Guide

This guide outlines the process for contributing new content, updating existing content, and ensuring quality for the Physical AI & Humanoid Robotics book. Your contributions help make this resource valuable for everyone.

## 1. Getting Started

Before you start, make sure you have:

*   **Node.js (v18.0+) and npm/yarn:** Installed on your machine.
*   **Git:** Version control system.
*   **A GitHub Account:** For submitting changes via Pull Requests.

Follow the [Quickstart Guide](/docs/quickstart) to set up your local development environment.

## 2. Project Structure

Familiarize yourself with the book's directory structure:

*   `book/docs/`: Contains all Markdown (`.md`) files for the book's chapters. Organized into modules.
*   `book/src/components/`: Custom React components used within Markdown.
*   `book/src/theme/`: Custom Docusaurus theme components.
*   `book/static/`: Static assets like images (`img/`) and videos (`videos/`).

## 3. Writing Guidelines

All content must adhere to the following guidelines:

### a. Content Structure

*   Each chapter should start with a clear title (H1) and a brief description in the frontmatter.
*   Break down topics into logical sections using headings (H2, H3).
*   Start chapters with clear learning objectives and end with summaries/key takeaways.
*   Ensure content can be understood without requiring a local ROS 2 runtime.

### b. Markdown Frontmatter

Each Markdown file **must** include the following YAML frontmatter:

```yaml
---
title: Your Chapter Title
description: A brief summary of the chapter.
module_number: [1-4, corresponding to the module]
duration_weeks: [Estimated time to complete this chapter, e.g., 1.5]
prerequisites: ["List", "of", "required", "knowledge"]
learning_objectives: ["List", "of", "skills", "gained"]
---
```

Refer to `book/docusaurus.config.ts` for the full frontmatter schema documentation.

### c. Code Examples

*   Use Docusaurus's built-in code blocks for syntax highlighting.
*   Provide context and explanation for each code example.
*   Include comments within the code where necessary.
*   Example:

    ```markdown
    ```python
    # This is a Python example
    def hello_robot():
        print("Hello, Robot!")
    ```
    ```

*   For multi-language examples, use `Tabs` and `TabItem` components as shown in `quickstart.md`.

### d. Diagrams and Images

*   Place all image files in `book/static/img/`.
*   Reference images using Markdown syntax: `![Alt text for accessibility](/img/your-image-name.png)`.
*   **Always include descriptive `alt text` for accessibility.**

## 4. Workflow

1.  **Create an Issue:** Before starting work, create a new GitHub issue to describe your proposed changes or new content. This allows for discussion and avoids duplicate effort.
2.  **Create a New Branch:**
    ```bash
    git checkout main
    git pull origin main
    git checkout -b feature/your-chapter-name
    ```
3.  **Add/Edit Content:** Create new Markdown files in the appropriate `book/docs/module-X/` directory or edit existing ones.
4.  **Update `sidebars.ts`:** If you add a new page, make sure to add its path to `book/sidebars.ts` in the correct module category so it appears in the navigation.
5.  **Test Locally:**
    ```bash
    cd book
    npm install # if new dependencies
    npm start
    ```
    Verify your changes locally by navigating to `http://localhost:3000`.
6.  **Run Validation:** Before committing, ensure your frontmatter is valid:
    ```bash
    cd book
    npm run prebuild
    ```
7.  **Commit Changes:**
    ```bash
    git add .
    git commit -m "feat: Add new chapter on [Chapter Title]"
    ```
    Write clear and concise commit messages.
8.  **Push to GitHub:**
    ```bash
    git push origin feature/your-chapter-name
    ```
9.  **Open a Pull Request (PR):**
    *   Go to your GitHub repository and open a new Pull Request from your feature branch to `main`.
    *   Provide a clear title and description for your PR, referencing the issue you created.
    *   Request a review from a maintainer.

## 5. Review Process

*   Maintainers will review your PR for content accuracy, adherence to guidelines, and technical correctness.
*   Be prepared to address feedback and make further revisions.

Thank you for contributing!
