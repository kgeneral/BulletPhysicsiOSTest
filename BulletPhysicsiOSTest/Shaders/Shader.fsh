//
//  Shader.fsh
//  BulletPhysicsiOSTest
//
//  Created by Dae-Yeong Kim on 13. 2. 26..
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//

varying lowp vec4 colorVarying;

void main()
{
    gl_FragColor = colorVarying;
}
