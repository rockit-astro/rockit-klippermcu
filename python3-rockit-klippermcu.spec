Name:           python3-rockit-klippermcu
Version:        %{_version}
Release:        1
Summary:        Code to control a microcontroller running the Klipper firmware
License:        GPL3
Url:            https://github.com/rockit-astro/rockit-klippermcu
BuildArch:      x86_64 aarch64
BuildRequires:  python3-devel

%description

%prep
rsync -av --exclude=build --exclude=.git --exclude=.github .. .

%generate_buildrequires
%pyproject_buildrequires -R

%build
%pyproject_wheel

%install
%pyproject_install
%pyproject_save_files rockit
make lib
mkdir -p %{buildroot}%{_libdir}
mv libklippermcu.so %{buildroot}%{_libdir}
ls %{buildroot}%{_libdir}

%files -f %{pyproject_files}
%{_libdir}/libklippermcu.so
