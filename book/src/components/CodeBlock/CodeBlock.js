import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const CodeBlock = ({ children, language = 'python', title = '' }) => {
  return (
    <BrowserOnly
      fallback={<div>Loading code block...</div>}
    >
      {() => {
        const { Highlight, themes } = require('prism-react-renderer');
        const code = children.trim();

        return (
          <div className="code-block-wrapper">
            {title && <div className="code-block-title">{title}</div>}
            <Highlight theme={themes.github} code={code} language={language}>
              {({ className, style, tokens, getLineProps, getTokenProps }) => (
                <pre className={className} style={style}>
                  {tokens.map((line, i) => (
                    <div key={i} {...getLineProps({ line })}>
                      {line.map((token, key) => (
                        <span key={key} {...getTokenProps({ token })} />
                      ))}
                    </div>
                  ))}
                </pre>
              )}
            </Highlight>
          </div>
        );
      }}
    </BrowserOnly>
  );
};

export default CodeBlock;