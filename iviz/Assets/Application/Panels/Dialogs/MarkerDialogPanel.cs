﻿#nullable enable

using System;
using Iviz.Core;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Iviz.App
{
    public sealed class MarkerDialogPanel : DetachableDialogPanel
    {
        [SerializeField] SimpleButtonWidget? close;
        [SerializeField] Button? reset;
        [SerializeField] DataLabelWidget? label;
        [SerializeField] TMP_Text? text;
        [SerializeField] LinkResolver? linkResolver;
        [SerializeField] Button? left;
        [SerializeField] Button? right;
        [SerializeField] TMP_Text? leftText;
        [SerializeField] TMP_Text? rightText;

        Button Reset => reset.AssertNotNull(nameof(reset));
        LinkResolver LinkResolver => linkResolver.AssertNotNull(nameof(linkResolver));
        Button Left => left.AssertNotNull(nameof(left));
        Button Right => right.AssertNotNull(nameof(right));
        TMP_Text LeftText => leftText.AssertNotNull(nameof(leftText));
        TMP_Text RightText => rightText.AssertNotNull(nameof(rightText));
        public SimpleButtonWidget Close => close.AssertNotNull(nameof(close));
        public DataLabelWidget Label => label.AssertNotNull(nameof(label));
        public TMP_Text Text => text.AssertNotNull(nameof(text));

        public event Action<int>? Flipped; 
        
        public string LeftCaption
        {
            set => LeftText.text = value;
        }

        public string RightCaption
        {
            set => RightText.text = value;
        }

        public event Action? ResetAll;
        public event Action<string>? LinkClicked;

        void Awake()
        {
            Reset.onClick.AddListener(() => ResetAll?.Invoke());
            LinkResolver.LinkClicked += s => LinkClicked?.Invoke(s);
            Left.onClick.AddListener(() => Flipped?.Invoke(-1));
            Right.onClick.AddListener(() => Flipped?.Invoke(1));
        }
        
        public override void ClearSubscribers()
        {
            Close.ClearSubscribers();
            ResetAll = null;
            LinkClicked = null;
            Flipped = null;
        }
    }
}
