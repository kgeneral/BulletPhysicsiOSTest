//
//  Shader.vsh
//  BulletPhysicsiOSTest
//
//  Created by Dae-Yeong Kim on 13. 2. 26..
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//

attribute vec4 position;
attribute vec3 normal;

varying lowp vec4 colorVarying;

uniform mat4 modelViewProjectionMatrix;
uniform mat3 normalMatrix;

void main()
{
    vec3 eyeNormal = normalize(normalMatrix * normal);
    vec3 lightPosition = vec3(2.0, 11.0, 1.0);
    vec4 diffuseColor = vec4(0.4, 0.9, 1.0, 1.0);
    
    float nDotVP = max(0.0, dot(eyeNormal, normalize(lightPosition)));
                 
    colorVarying = diffuseColor * nDotVP;
    
    gl_Position = modelViewProjectionMatrix * position;
}
