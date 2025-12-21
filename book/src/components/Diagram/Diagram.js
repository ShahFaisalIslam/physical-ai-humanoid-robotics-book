import React from 'react';
import clsx from 'clsx';

const Diagram = ({ src, alt, caption, className }) => {
  return (
    <div className={clsx('diagram-container', className)}>
      <img src={src} alt={alt} className="diagram-image" />
      {caption && <div className="diagram-caption">{caption}</div>}
    </div>
  );
};

export default Diagram;