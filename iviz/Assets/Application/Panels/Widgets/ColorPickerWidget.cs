﻿using System;
using Iviz.Core;
using Iviz.Resources;
using JetBrains.Annotations;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Iviz.App
{
    public sealed class ColorPickerWidget : MonoBehaviour, IWidget
    {
        [SerializeField] SliderWidget sliderX;
        [SerializeField] SliderWidget sliderY;
        [SerializeField] SliderWidget sliderZ;
        //[SerializeField] Text label = null;
        [SerializeField] TMP_Text label;
        [SerializeField] Button button;
        [SerializeField] Image panel;
        Color color;

        enum ColorMode
        {
            RGB,
            HSV
        }

        ColorMode colorMode;
        bool disableUpdates;

        [NotNull]
        public string Label
        {
            get => label.text;
            set
            {
                ThrowHelper.ThrowIfNull(value, nameof(value));
                name = "ColorPicker:" + value;
                label.text = value;
            }
        }

        public Color Value
        {
            get => color;
            set
            {
                color = value;
                UpdateSliderLabels();
                UpdateColorButton();
            }
        }

        public bool Interactable
        {
            get => sliderX.Interactable;
            set
            {
                sliderX.Interactable = value;
                sliderY.Interactable = value;
                sliderZ.Interactable = value;
                button.interactable = value;
                label.color = value ? Resource.Colors.FontEnabled : Resource.Colors.FontDisabled;
                panel.color = value ? Resource.Colors.EnabledPanel : Resource.Colors.DisabledPanel;
            }
        }

        public event Action<Color> ValueChanged;

        void OnValueChanged()
        {
            if (disableUpdates)
            {
                return;
            }

            switch (colorMode)
            {
                case ColorMode.RGB:
                    color = new Color(sliderX.Value, sliderY.Value, sliderZ.Value);
                    break;
                case ColorMode.HSV:
                    color = Color.HSVToRGB(sliderX.Value, sliderY.Value, sliderZ.Value);
                    break;
            }

            UpdateColorButton();
            ValueChanged?.Invoke(color);
        }

        void UpdateColorButton()
        {
            ColorBlock colorBlock = button.colors;
            colorBlock.highlightedColor = color;
            colorBlock.normalColor = color;
            colorBlock.selectedColor = color;
            colorBlock.pressedColor = color / 2;
            button.colors = colorBlock;
        }

        void UpdateSliderLabels()
        {
            disableUpdates = true;
            switch (colorMode)
            {
                case ColorMode.RGB:
                    sliderX.Value = color.r;
                    sliderY.Value = color.g;
                    sliderZ.Value = color.b;
                    break;
                case ColorMode.HSV:
                    Color.RGBToHSV(color, out var h, out var s, out var v);
                    sliderX.Value = h;
                    sliderY.Value = s;
                    sliderZ.Value = v;
                    break;
            }

            disableUpdates = false;
        }

        void SwitchColorMode()
        {
            switch (colorMode)
            {
                case ColorMode.RGB:
                    colorMode = ColorMode.HSV;
                    sliderX.Label = "H";
                    sliderY.Label = "S";
                    sliderZ.Label = "V";
                    break;
                case ColorMode.HSV:
                    colorMode = ColorMode.RGB;
                    sliderX.Label = "R";
                    sliderY.Label = "G";
                    sliderZ.Label = "B";
                    break;
            }

            UpdateSliderLabels();
        }

        public void ClearSubscribers()
        {
            ValueChanged = null;
        }

        [NotNull]
        public ColorPickerWidget SetLabel([NotNull] string f)
        {
            Label = f;
            return this;
        }

        [NotNull]
        public ColorPickerWidget SetValue(Color f)
        {
            Value = f;
            return this;
        }

        [NotNull]
        public ColorPickerWidget SetInteractable(bool f)
        {
            Interactable = f;
            return this;
        }

        [NotNull]
        public ColorPickerWidget SubscribeValueChanged(Action<Color> f)
        {
            ValueChanged += f;
            return this;
        }

        void Awake()
        {
            sliderX.SetMinValue(0).SetMaxValue(1).SetNumberOfSteps(256).SubscribeValueChanged(f => OnValueChanged());
            sliderY.SetMinValue(0).SetMaxValue(1).SetNumberOfSteps(256).SubscribeValueChanged(f => OnValueChanged());
            sliderZ.SetMinValue(0).SetMaxValue(1).SetNumberOfSteps(256).SubscribeValueChanged(f => OnValueChanged());
            button.onClick.AddListener(SwitchColorMode);
        }

        void Start()
        {
            Value = color;
            OnValueChanged();
        }
    }
}