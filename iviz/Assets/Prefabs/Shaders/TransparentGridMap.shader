﻿Shader "iviz/TransparentGridMap"
{
    Properties
    {
        _IntensityTex("Atlas Texture", 2D) = "defaulttexture" {}
        _SquareTex("Square Texture", 2D) = "defaulttexture" {}
    }

    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent"}
        
        CGPROGRAM
        #pragma surface surf Standard vertex:vert addshadow fullforwardshadows alpha:fade

        #pragma multi_compile _ USE_NORMALS
        
        float _AtlasRow;
        sampler2D _SquareTex;
        sampler2D _InputTex;
        sampler2D _IntensityTex;

        float _IntensityCoeff;
        float _IntensityAdd;
        float4 _Tint;
        float _Metallic;
        float _Smoothness;

        float4 _SquareCoeff;

        struct Input
        {
            float2 squareTextureUV : TEXCOORD0;
            float2 intensityUV : TEXCOORD1;
        };

        void vert(inout appdata_full v, out Input o)
        {
            UNITY_INITIALIZE_OUTPUT(Input, o);

            float2 uv = 1 - v.texcoord;
            float input = tex2Dlod(_InputTex, float4(uv, 0, 0));

            v.vertex.y = input;

#if USE_NORMALS
            const float2 normalCoeff = _SquareCoeff.xy;
            const float2 normalOffset = _SquareCoeff.zw;
            float px = tex2Dlod(_InputTex, float4(uv.x + normalOffset.x, uv.y, 0, 0));
            float py = tex2Dlod(_InputTex, float4(uv.x, uv.y + normalOffset.y, 0, 0));

            float3 f;
            f.x = (py - input) * normalCoeff.x;
            f.y = 1;
            f.z = -(px -  input) * normalCoeff.y;

            if (isnan(f.x)) f.x = 0;
            if (isnan(f.z)) f.z = 0;

            v.normal = normalize(f);
#endif
            
            o.intensityUV = float2(input * _IntensityCoeff + _IntensityAdd, _AtlasRow);
            o.squareTextureUV = uv * _SquareCoeff.xy;
        }

        void surf(Input IN, inout SurfaceOutputStandard o)
        {
            o.Albedo =
                tex2D(_IntensityTex, IN.intensityUV) *
                tex2D(_SquareTex, IN.squareTextureUV) *
                _Tint;
            o.Alpha = _Tint.a;
            o.Metallic = _Metallic;
            o.Smoothness = _Smoothness;
        }
        ENDCG
    }
}