# Markdown Syntax Examples

## Text Formatting

- **Bold**:  
  ```
  **Bold Text**
  ```
  **Bold Text**

- *Italic*:  
  ```
  *Italic Text*
  ```
  *Italic Text*

- ***Bold and Italic***:  
  ```
  ***Bold and Italic Text***
  ```
  ***Bold and Italic Text***

- ~~Strikethrough~~:  
  ```
  ~~Strikethrough Text~~
  ```
  ~~Strikethrough Text~~

## Lists

- Unordered List:  
  ```
  - Item 1
  - Item 2
  ```
  - Item 1
  - Item 2

- Ordered List:  
  ```
  1. First Item
  2. Second Item
  ```
  1. First Item
  2. Second Item

## Links and Images

- Link:  
  ```
  [Link Text](https://www.example.com)
  ```
  [Link Text](https://www.example.com)

- Image:  
  ```
  ![Alt Text](https://www.example.com/image.jpg)
  ```
  ![Alt Text](https://www.example.com/image.jpg)

## Code

### Inline code
  ```
  `code`
  ```
  `code`

### Code Block  
  ```
  \`\`\`
  Code block here
  \`\`\`
  ```
  ```
  Code block here
  ```

### Syntax-highlighted Code Blocks  

- Python:  
  ```
  \`\`\`python
  print("Hello, world!")
  \`\`\`
  ```
  ```python
  print("Hello, world!")
  ```

- C++:  
  ```
  \`\`\`cpp
  #include <iostream>
  int main() {
      std::cout << "Hello, world!" << std::endl;
      return 0;
  }
  \`\`\`
  ```
  ```cpp
  #include <iostream>
  int main() {
      std::cout << "Hello, world!" << std::endl;
      return 0;
  }
  ```

- JavaScript:  
  ```
  \`\`\`javascript
  console.log("Hello, world!");
  \`\`\`
  ```
  ```javascript
  console.log("Hello, world!");
  ```

- Bash:  
  ```
  \`\`\`bash
  echo "Hello, world!"
  \`\`\`
  ```
  ```bash
  echo "Hello, world!"
  ```

## Quotes

- Blockquote:  
  ```
  > This is a quote.
  ```
  > This is a quote.

## Horizontal Rule

  ```
  ---
  ```
  ---

## Headers

- Main Header:  
  ```
  # Header 1
  ```
  # Header 1

- Sub-header:  
  ```
  ## Header 2
  ```
  ## Header 2

- Sub-sub-header:  
  ```
  ### Header 3
  ```
  ### Header 3

- Additional Heading Levels:  
  ```
  #### Header 4
  ##### Header 5
  ###### Header 6
  ```
  #### Header 4
  ##### Header 5
  ###### Header 6

## Tables

  ```
  | Header 1 | Header 2 |
  |----------|----------|
  | Cell 1   | Cell 2   |
  | Cell 3   | Cell 4   |
  ```
  | Header 1 | Header 2 |
  |----------|----------|
  | Cell 1   | Cell 2   |
  | Cell 3   | Cell 4   |

## Task Lists

  ```
  - [x] Completed Task
  - [ ] Incomplete Task
  ```
  - [x] Completed Task
  - [ ] Incomplete Task

## Escaping Special Characters

- To use any of the Markdown special characters (like `*`, `_`, `#`, etc.) as literal characters, you can escape them with a backslash (\).  
  ```
  \*This is not italic\*
  ```
  \*This is not italic\*
