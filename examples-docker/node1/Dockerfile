#See https://aka.ms/customizecontainer to learn how to customize your debug container and how Visual Studio uses this Dockerfile to build your images for faster debugging.

FROM mcr.microsoft.com/dotnet/runtime:7.0 AS base
WORKDIR /app

FROM mcr.microsoft.com/dotnet/sdk:7.0 AS build
WORKDIR /src
COPY ["node1/node1.csproj", "node1/"]
RUN dotnet restore "node1/node1.csproj"
COPY . .
WORKDIR "/src/node1"
RUN dotnet build "node1.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish --self-contained --runtime linux-x64 "node1.csproj" -c Release -o /app/publish

FROM ros:humble-ros-core AS final
WORKDIR /app
COPY --from=publish /app/publish .
ENTRYPOINT . /opt/ros/humble/setup.sh && ./node1