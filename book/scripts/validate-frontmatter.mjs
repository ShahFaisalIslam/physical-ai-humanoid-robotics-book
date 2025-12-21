import { readFileSync } from 'fs';
import { globSync } from 'glob';
import grayMatter from 'gray-matter';

// Define the schema for validation
const frontmatterSchema = {
  module_number: { type: 'number', required: true, min: 1, max: 4 },
  duration_weeks: { type: 'number', required: true, min: 0.1 },
  prerequisites: { type: 'array', of: 'string', required: true },
  learning_objectives: { type: 'array', of: 'string', required: true },
};

console.log('Starting Docusaurus frontmatter validation...');
let hasErrors = false;

const markdownFiles = globSync('book/docs/**/*.md', { absolute: false });

for (const filePath of markdownFiles) {
  const fileContent = readFileSync(filePath, 'utf-8');
  const { data: frontmatter } = grayMatter(fileContent);

  // Validate each field in the schema
  for (const field in frontmatterSchema) {
    const schema = frontmatterSchema[field];
    const value = frontmatter[field];

    if (schema.required && (value === undefined || value === null)) {
      console.error(`ERROR: ${filePath}: Missing required frontmatter field: "${field}"`);
      hasErrors = true;
      continue;
    }

    if (value !== undefined && value !== null) {
      if (schema.type === 'number') {
        if (typeof value !== 'number') {
          console.error(`ERROR: ${filePath}: Field "${field}" must be a number. Found: "${typeof value}"`);
          hasErrors = true;
        } else {
          if (schema.min !== undefined && value < schema.min) {
            console.error(`ERROR: ${filePath}: Field "${field}" must be at least ${schema.min}. Found: ${value}`);
            hasErrors = true;
          }
          if (schema.max !== undefined && value > schema.max) {
            console.error(`ERROR: ${filePath}: Field "${field}" must be at most ${schema.max}. Found: ${value}`);
            hasErrors = true;
          }
        }
      } else if (schema.type === 'array') {
        if (!Array.isArray(value)) {
          console.error(`ERROR: ${filePath}: Field "${field}" must be an array. Found: "${typeof value}"`);
          hasErrors = true;
        } else if (schema.of === 'string' && !value.every(item => typeof item === 'string')) {
          console.error(`ERROR: ${filePath}: Field "${field}" array must contain only strings.`);
          hasErrors = true;
        }
      }
    }
  }
}

if (hasErrors) {
  console.error('Frontmatter validation FAILED. Please fix the errors above.');
  process.exit(1);
} else {
  console.log('Frontmatter validation PASSED.');
}
