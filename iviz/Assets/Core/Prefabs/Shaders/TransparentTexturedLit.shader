﻿Shader "iviz/TransparentTexturedLit"
{
    Properties
    {
        _MainTex("Color Texture", 2D) = "white" {}
    }
    SubShader
    {
        Tags
        {
            "Queue"="Transparent" "RenderType"="Transparent"
        }
        LOD 200

        CGPROGRAM
        #pragma surface surf Standard fullforwardshadows addshadow alpha:fade

        sampler2D _MainTex;

        struct Input
        {
            float4 color : COLOR;
            float2 uv_MainTex;
        };

        UNITY_INSTANCING_BUFFER_START(Props)
        UNITY_DEFINE_INSTANCED_PROP(fixed4, _Color)
        //UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST_)
        UNITY_DEFINE_INSTANCED_PROP(fixed4, _EmissiveColor)
        UNITY_DEFINE_INSTANCED_PROP(half, _Metallic)
        UNITY_DEFINE_INSTANCED_PROP(half, _Smoothness)
        UNITY_INSTANCING_BUFFER_END(Props)

        void surf(Input IN, inout SurfaceOutputStandard o)
        {
            const fixed4 albedo_color = UNITY_ACCESS_INSTANCED_PROP(Props, _Color) * IN.color;
            //const float4 st = UNITY_ACCESS_INSTANCED_PROP(Props, _MainTex_ST_);
            //const fixed4 texture_color = tex2D(_MainTex, IN.uv_MainTex * st.xy + st.zw);
            const fixed4 texture_color = tex2D(_MainTex, IN.uv_MainTex);
            o.Albedo = albedo_color.rgb * texture_color.rgb;
            o.Alpha = albedo_color.a * texture_color.a;
            o.Metallic = UNITY_ACCESS_INSTANCED_PROP(Props, _Metallic);
            o.Smoothness = UNITY_ACCESS_INSTANCED_PROP(Props, _Smoothness);
            o.Emission = UNITY_ACCESS_INSTANCED_PROP(Props, _EmissiveColor).rgb;
        }
        ENDCG
    }
}