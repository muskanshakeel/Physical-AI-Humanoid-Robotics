import React from 'react';
import SSRCompatibleAuthButtons from '@site/src/components/AuthButtons/SSRCompatibleAuthButtons';

const AuthNavbarItem = (props) => {
  return (
    <div {...props}>
      <SSRCompatibleAuthButtons />
    </div>
  );
};

export default AuthNavbarItem;