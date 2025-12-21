# Data Model: Physical AI & Humanoid Robotics Book

## Overview
This document defines the key data structures and entities for the Physical AI & Humanoid Robotics book project using Docusaurus. These entities represent the core concepts that will be taught and demonstrated in the book.

## Key Entities

### 1. BookChapter
Represents a major section of the book, corresponding to one of the four main modules.

**Fields:**
- id: string (unique identifier, e.g., "module-1-ros-fundamentals")
- title: string (chapter title)
- description: string (brief overview of the chapter content)
- module_number: integer (1-4, indicating which module this belongs to)
- duration_weeks: float (estimated time to complete, in weeks)
- prerequisites: array of strings (knowledge required before reading)
- learning_objectives: array of strings (skills readers will gain)
- content_sections: array of ContentSection objects
- code_examples: array of CodeExample objects
- exercises: array of Exercise objects
- assessments: array of Assessment objects

**Relationships:**
- Contains many ContentSection entities
- Contains many CodeExample entities
- Contains many Exercise entities
- Contains many Assessment entities

### 2. ContentSection
Represents a subsection within a chapter that covers a specific topic.

**Fields:**
- id: string (unique identifier)
- title: string (section title)
- content_type: string (text, video, interactive, etc.)
- content: string (the actual content in markdown format)
- estimated_reading_time: integer (in minutes)
- difficulty_level: string (beginner, intermediate, advanced)
- related_topics: array of strings (other topics related to this section)

**Relationships:**
- Belongs to one BookChapter entity
- May reference multiple Resource entities

### 3. CodeExample
Represents code samples included in the book.

**Fields:**
- id: string (unique identifier)
- title: string (brief description of the example)
- description: string (explanation of what the code does)
- language: string (programming language, typically "python" or "bash")
- code: string (actual code content)
- dependencies: array of strings (ROS packages, Python libraries, etc.)
- explanation: string (detailed explanation of the code)
- use_case: string (where this code would be applied in robotics)
- complexity_level: string (simple, moderate, complex)

**Relationships:**
- Belongs to one BookChapter entity
- May reference multiple Resource entities

### 4. Exercise
Practical tasks for readers to complete to reinforce learning.

**Fields:**
- id: string (unique identifier)
- title: string (exercise title)
- description: string (detailed instructions)
- difficulty_level: string (beginner, intermediate, advanced)
- estimated_completion_time: integer (in minutes)
- required_resources: array of strings (reading materials, code examples)
- evaluation_criteria: array of strings (how the exercise will be assessed)
- solution_outline: string (outline of the solution approach)
- hints: array of strings (guidance for readers struggling with the exercise)

**Relationships:**
- Belongs to one BookChapter entity
- May reference multiple CodeExample entities
- May reference multiple Resource entities

### 5. Assessment
Evaluation mechanisms to test reader comprehension.

**Fields:**
- id: string (unique identifier)
- title: string (assessment title)
- type: string (quiz, practical, project, etc.)
- description: string (what the assessment evaluates)
- questions: array of AssessmentQuestion objects
- passing_score: float (minimum score required)
- time_limit_minutes: integer (if applicable)
- feedback_template: string (how feedback is provided to readers)

**Relationships:**
- Belongs to one BookChapter entity
- Contains many AssessmentQuestion entities

### 6. AssessmentQuestion
Individual questions within an assessment.

**Fields:**
- id: string (unique identifier)
- question_text: string (the actual question)
- question_type: string (multiple_choice, short_answer, practical_task, etc.)
- options: array of strings (for multiple choice questions)
- correct_answer: string (the correct response)
- explanation: string (why this is the correct answer)
- difficulty_level: string (beginner, intermediate, advanced)

**Relationships:**
- Belongs to one Assessment entity

### 7. Resource
External materials referenced in the book.

**Fields:**
- id: string (unique identifier)
- title: string (resource title)
- url: string (web address)
- type: string (documentation, video, paper, code_repo, etc.)
- description: string (what this resource provides)
- relevance: string (how it relates to book content)
- tags: array of strings (topics this resource covers)

**Relationships:**
- Referenced by multiple ContentSection entities
- Referenced by multiple CodeExample entities
- Referenced by multiple Exercise entities

### 8. Diagram
Visual representations used in the book.

**Fields:**
- id: string (unique identifier)
- title: string (diagram title)
- description: string (what the diagram illustrates)
- file_path: string (path to the image file)
- alt_text: string (accessibility text)
- tags: array of strings (topics this diagram covers)

**Relationships:**
- Referenced by multiple ContentSection entities
- Referenced by multiple BookChapter entities

### 9. InteractiveDemo
Interactive elements in the book (if implemented).

**Fields:**
- id: string (unique identifier)
- title: string (demo title)
- description: string (what the demo demonstrates)
- component_path: string (path to the React component)
- props_schema: object (schema for component properties)
- use_case: string (where this demo is used in the book)

**Relationships:**
- Referenced by multiple ContentSection entities
- Belongs to one BookChapter entity

### 10. BookConfiguration
Configuration settings for the Docusaurus book.

**Fields:**
- id: string (unique identifier)
- site_title: string (title of the book site)
- tagline: string (subtitle or tagline)
- favicon: string (path to favicon)
- theme_config: object (Docusaurus theme configuration)
- sidebar_config: object (navigation sidebar configuration)
- deployment_config: object (GitHub Pages deployment settings)

## State Transitions

### BookChapter States
- draft → review → approved → published
- published → updated → approved → republished

### ContentSection States
- idea → writing → review → approved → published
- published → revised → review → approved → republished

## Validation Rules

1. Each BookChapter must have at least one ContentSection
2. Each CodeExample must have a valid language field
3. Each Assessment must have at least one AssessmentQuestion
4. Each diagram must have appropriate alt_text for accessibility
5. Difficulty levels must be one of: "beginner", "intermediate", "advanced"
6. Estimated times must be positive numbers
7. IDs must be unique within their entity type