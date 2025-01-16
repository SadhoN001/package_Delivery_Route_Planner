from django.urls import path
from . import views



urlpatterns = [
    path('', views.route_planner, name='route_planner'),
] 