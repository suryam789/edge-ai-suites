# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for Weather Service."""

import pytest
import asyncio
from unittest.mock import Mock, patch, MagicMock, AsyncMock
from datetime import datetime, timedelta, timezone
import json
import os
import sys

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from services.weather_service import WeatherService
from services.config import ConfigService
from models.weather import WeatherData
from models.enums import WeatherType


class TestWeatherServiceInitialization:
    """Test cases for WeatherService initialization."""

    def test_init_with_default_config(self):
        """Test WeatherService initializes with default configuration."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        assert service.api_base_url == "https://api.weather.gov"
        assert service.use_mock is False
        assert service._cached_weather is None
        assert service._running is False

    def test_init_with_custom_config(self):
        """Test WeatherService initializes with custom configuration."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {
            "api_base_url": "https://custom.api.url",
            "cache_duration_minutes": 30,
            "update_interval_minutes": 15,
            "use_mock": True
        }
        
        service = WeatherService(mock_config)
        
        assert service.api_base_url == "https://custom.api.url"
        assert service.cache_duration == timedelta(minutes=30)
        assert service.update_interval == timedelta(minutes=15)
        assert service.use_mock is True

    def test_init_with_mock_mode_enabled(self):
        """Test WeatherService initializes with mock mode."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"use_mock": True}
        
        service = WeatherService(mock_config)
        
        assert service.use_mock is True


class TestWeatherServiceCaching:
    """Test cases for WeatherService caching functionality."""

    def test_cache_valid_within_duration(self):
        """Test cache is valid when within cache duration."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"cache_duration_minutes": 15}
        
        service = WeatherService(mock_config)
        service._cached_weather = Mock()
        service._cache_timestamp = datetime.now(timezone.utc) - timedelta(minutes=5)
        
        assert service._is_cache_valid() is True

    def test_cache_invalid_after_expiry(self):
        """Test cache is invalid after cache duration expires."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"cache_duration_minutes": 15}
        
        service = WeatherService(mock_config)
        service._cached_weather = Mock()
        service._cache_timestamp = datetime.now(timezone.utc) - timedelta(minutes=20)
        
        assert service._is_cache_valid() is False

    def test_cache_invalid_when_empty(self):
        """Test cache is invalid when no cached data exists."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        assert service._is_cache_valid() is False

    def test_cache_invalid_when_no_timestamp(self):
        """Test cache is invalid when timestamp is missing."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        service._cached_weather = Mock()
        service._cache_timestamp = None
        
        assert service._is_cache_valid() is False


class TestWeatherServiceProcessing:
    """Test cases for weather data processing."""

    def test_process_weather_data_basic(self):
        """Test processing basic weather forecast data."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "name": "This Afternoon",
            "temperature": 75,
            "temperatureUnit": "F",
            "shortForecast": "Sunny",
            "detailedForecast": "Sunny with clear skies.",
            "windSpeed": "10 mph",
            "windDirection": "NW",
            "probabilityOfPrecipitation": {"value": 5},
            "isDaytime": True,
            "startTime": "2026-01-22T12:00:00-08:00",
            "endTime": "2026-01-22T13:00:00-08:00"
        }
        
        result = service._process_weather_data(forecast_period)
        
        assert result.temperature == 75
        assert result.temperature_unit == "F"
        assert result.short_forecast == "Sunny"
        assert result.wind_speed == "10 mph"
        assert result.wind_direction == "NW"
        assert result.precipitation_prob == 5.0
        assert result.is_precipitation is False  # 5% < 30%
        assert result.is_mock is False

    def test_process_weather_data_high_precipitation(self):
        """Test processing weather data with high precipitation probability."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "temperature": 55,
            "temperatureUnit": "F",
            "shortForecast": "Rainy",
            "detailedForecast": "Rain expected throughout the day.",
            "windSpeed": "15 mph",
            "windDirection": "S",
            "probabilityOfPrecipitation": {"value": 80}
        }
        
        result = service._process_weather_data(forecast_period)
        
        assert result.is_precipitation is True  # 80% > 30%
        assert result.precipitation_prob == 80.0

    def test_process_weather_data_empty_detailed_forecast(self):
        """Test processing when detailed forecast is empty."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "temperature": 65,
            "temperatureUnit": "F",
            "shortForecast": "Cloudy",
            "detailedForecast": "",
            "windSpeed": "5 mph",
            "windDirection": "E",
            "probabilityOfPrecipitation": {"value": 20}
        }
        
        result = service._process_weather_data(forecast_period)
        
        # Should construct detailed forecast from available data
        assert "Cloudy" in result.detailed_forecast
        assert "5 mph" in result.detailed_forecast
        assert "E" in result.detailed_forecast
        assert "20%" in result.detailed_forecast

    def test_process_weather_data_missing_precipitation(self):
        """Test processing when precipitation data is missing."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "temperature": 70,
            "temperatureUnit": "F",
            "shortForecast": "Clear",
            "windSpeed": "3 mph",
            "windDirection": "N"
        }
        
        result = service._process_weather_data(forecast_period)
        
        assert result.precipitation_prob == 0.0
        assert result.is_precipitation is False

    def test_process_weather_data_wind_info_format(self):
        """Test wind_info field is formatted correctly."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "temperature": 68,
            "temperatureUnit": "F",
            "shortForecast": "Partly Cloudy",
            "windSpeed": "12 mph",
            "windDirection": "SW",
            "probabilityOfPrecipitation": {"value": 0}
        }
        
        result = service._process_weather_data(forecast_period)
        
        assert result.wind_info == "12mph/SW"


class TestWeatherServiceMockData:
    """Test cases for mock weather data functionality."""

    def test_get_default_weather(self):
        """Test getting default weather when no data is available."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        result = service.get_default_weather()
        
        assert result.name == "Unknown"
        assert result.temperature == 72
        assert result.temperature_unit == "F"
        assert result.is_mock is True
        assert "unavailable" in result.detailed_forecast.lower()

    def test_load_mock_weather_file_not_found(self):
        """Test loading mock weather when file doesn't exist."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        service.mock_data_file = "/nonexistent/path/weather.json"
        
        result = service._load_mock_weather_from_file()
        
        # Should return default weather
        assert result.is_mock is True
        assert result.temperature == 72

    @patch('builtins.open')
    @patch('os.path.exists')
    def test_load_mock_weather_from_valid_file(self, mock_exists, mock_open):
        """Test loading mock weather from a valid JSON file."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        mock_exists.return_value = True
        mock_weather_data = {
            "clear": {
                "name": "Mock Clear",
                "temperature": 65,
                "temperature_unit": "F",
                "detailed_forecast": "Clear and sunny",
                "is_precipitation": False,
                "wind_speed": "5 mph",
                "wind_direction": "N"
            }
        }
        mock_open.return_value.__enter__ = Mock(return_value=Mock(
            read=Mock(return_value=json.dumps(mock_weather_data))
        ))
        mock_open.return_value.__exit__ = Mock(return_value=False)
        
        with patch('json.load', return_value=mock_weather_data):
            service = WeatherService(mock_config)
            result = service._load_mock_weather_from_file(WeatherType.CLEAR)
        
        assert result.is_mock is True
        assert result.temperature == 65
        assert result.name == "Mock Clear"


class TestWeatherServiceAsync:
    """Test cases for async weather service operations."""

    @pytest.mark.asyncio
    async def test_get_current_weather_returns_cached(self):
        """Test get_current_weather returns cached data when valid."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"cache_duration_minutes": 15}
        
        service = WeatherService(mock_config)
        
        # Set up valid cached data
        cached_weather = WeatherData(
            name="Cached",
            temperature=70,
            temperature_unit="F",
            detailed_forecast="Cached weather",
            fetched_at=datetime.now(timezone.utc),
            is_mock=False
        )
        service._cached_weather = cached_weather
        service._cache_timestamp = datetime.now(timezone.utc) - timedelta(minutes=5)
        
        result = await service.get_current_weather(force_refresh=False)
        
        assert result.name == "Cached"
        assert result.is_cached is True

    @pytest.mark.asyncio
    async def test_get_current_weather_force_refresh_with_mock(self):
        """Test get_current_weather with force_refresh in mock mode."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"use_mock": True}
        
        service = WeatherService(mock_config)
        
        with patch.object(service, '_load_mock_weather_from_file') as mock_load:
            mock_load.return_value = WeatherData(
                name="Mock",
                temperature=72,
                temperature_unit="F",
                detailed_forecast="Mock weather",
                fetched_at=datetime.now(timezone.utc),
                is_mock=True
            )
            
            result = await service.get_current_weather(force_refresh=True)
            
            mock_load.assert_called_once()
            assert result.is_mock is True

    @pytest.mark.asyncio
    async def test_start_service(self):
        """Test starting the weather service."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"use_mock": True}
        mock_config.get_intersection_coordinates.return_value = (37.7749, -122.4194)
        
        service = WeatherService(mock_config)
        
        with patch.object(service, 'get_current_weather', new_callable=AsyncMock) as mock_get:
            mock_get.return_value = Mock()
            
            # Start service
            await service.start()
            
            assert service._running is True
            mock_get.assert_called_once_with(force_refresh=True)
            
            # Clean up
            await service.stop()

    @pytest.mark.asyncio
    async def test_stop_service(self):
        """Test stopping the weather service."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"use_mock": True}
        
        service = WeatherService(mock_config)
        service._running = True
        service._update_task = asyncio.create_task(asyncio.sleep(100))
        
        await service.stop()
        
        assert service._running is False

    @pytest.mark.asyncio
    async def test_start_service_already_running(self):
        """Test starting service when already running does nothing."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        service._running = True
        
        await service.start()
        
        # Should still be running, no error thrown
        assert service._running is True


class TestWeatherServiceDescription:
    """Test cases for weather description methods."""

    def test_get_current_weather_description_with_cache(self):
        """Test getting weather description when cached data exists."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        service._cached_weather = WeatherData(
            name="Test",
            temperature=75,
            temperature_unit="F",
            detailed_forecast="Sunny and warm with light winds.",
            fetched_at=datetime.now(timezone.utc)
        )
        
        result = service.get_current_weather_description()
        
        assert result == "Sunny and warm with light winds."

    def test_get_current_weather_description_without_cache(self):
        """Test getting weather description when no cached data."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        service._cached_weather = None
        
        result = service.get_current_weather_description()
        
        assert result == "Unknown weather conditions"


class TestWeatherServiceEdgeCases:
    """Test cases for edge cases and error handling."""

    def test_process_weather_data_none_precipitation_value(self):
        """Test processing when precipitation value is None."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "temperature": 60,
            "temperatureUnit": "F",
            "shortForecast": "Overcast",
            "windSpeed": "8 mph",
            "windDirection": "W",
            "probabilityOfPrecipitation": {"value": None}
        }
        
        result = service._process_weather_data(forecast_period)
        
        assert result.precipitation_prob == 0.0
        assert result.is_precipitation is False

    def test_process_weather_data_string_precipitation(self):
        """Test processing when precipitation is not a dict."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "temperature": 60,
            "temperatureUnit": "F",
            "shortForecast": "Clear",
            "windSpeed": "5 mph",
            "windDirection": "N",
            "probabilityOfPrecipitation": "10%"  # String instead of dict
        }
        
        result = service._process_weather_data(forecast_period)
        
        # Should handle gracefully and return 0
        assert result.precipitation_prob == 0.0

    def test_process_weather_data_with_humidity_and_dewpoint(self):
        """Test processing weather data with humidity and dewpoint."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {}
        
        service = WeatherService(mock_config)
        
        forecast_period = {
            "temperature": 70,
            "temperatureUnit": "F",
            "shortForecast": "Humid",
            "windSpeed": "3 mph",
            "windDirection": "SE",
            "probabilityOfPrecipitation": {"value": 15},
            "dewpoint": {"value": 18.5},
            "relativeHumidity": {"value": 75}
        }
        
        result = service._process_weather_data(forecast_period)
        
        assert result.dewpoint == 18.5
        assert result.relative_humidity == 75

    def test_cache_boundary_exactly_at_expiry(self):
        """Test cache validity at exact expiry boundary."""
        mock_config = Mock(spec=ConfigService)
        mock_config.get_weather_config.return_value = {"cache_duration_minutes": 15}
        
        service = WeatherService(mock_config)
        service._cached_weather = Mock()
        # Set timestamp to exactly 15 minutes ago
        service._cache_timestamp = datetime.now(timezone.utc) - timedelta(minutes=15)
        
        # Cache should be invalid at exactly the boundary
        assert service._is_cache_valid() is False


class TestWeatherServiceFixtures:
    """Test cases using shared fixtures."""
    
    def test_weather_parsing(self, sample_weather_data):
        """Test weather parsing with sample fixture data."""
        # sample_weather_data fixture is automatically injected from conftest.py
        assert sample_weather_data["temperature"] == 55
        assert sample_weather_data["short_forecast"] == "Sunny"
        assert sample_weather_data["is_precipitation"] is False
