import React from 'react';
import clsx from 'clsx';
import { useThemeConfig } from '@docusaurus/theme-common';
import styles from './CustomFooter.module.css';

function CustomFooter() {
  const { footer } = useThemeConfig();
  
  if (!footer) {
    return null;
  }

  const { copyright, links = [], style } = footer;

  return (
    <footer
      className={clsx('footer', {
        'footer--dark': style === 'dark',
      })}
    >
      <div className="container container-fluid">
        {links && links.length > 0 && (
          <div className="footer__links">
            <div className="row">
              {links.map((item, key) => (
                <div key={key} className="col col--3">
                  <h4 className="footer__title">{item.title}</h4>
                  <ul className="footer__items">
                    {item.items?.map((linkItem, i) => (
                      <li key={i}>
                        <a
                          className="footer__link-item"
                          {...linkItem}
                        >
                          {linkItem.label}
                        </a>
                      </li>
                    ))}
                  </ul>
                </div>
              ))}
            </div>
          </div>
        )}
        {copyright ? (
          <div className="footer__copyright">
            <div className="text--center">
              {copyright}
            </div>
          </div>
        ) : null}
      </div>
    </footer>
  );
}

export default CustomFooter;